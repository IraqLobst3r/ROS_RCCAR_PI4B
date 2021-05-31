#include <condition_variable>
#include <iostream>
#include <linux/v4l2-controls.h>
#include <memory>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <stdio.h>
#include <string.h>
#include <thread>
#include <time.h>
#include <unistd.h>
#include <vector>

extern "C" {
#include "libavdevice/avdevice.h"
#include "libavutil/imgutils.h"
#include "libswscale/swscale.h"

#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavfilter/buffersink.h>
#include <libavfilter/buffersrc.h>
#include <libavutil/opt.h>
}

#include "arducam_mipicamera.h"

#include "custom_interfaces/msg/h264_image.hpp"
#include "rclcpp/rclcpp.hpp"

#define VCOS_ALIGN_DOWN(p, n) (((ptrdiff_t)(p)) & ~((n)-1))
#define VCOS_ALIGN_UP(p, n) VCOS_ALIGN_DOWN((ptrdiff_t)(p) + (n)-1, (n))

std::mutex mutex_run;
bool run = true;

class ArducamStereoNode : public rclcpp::Node {
  public:
    ArducamStereoNode() : Node("arducam_stereo_node") {

        this->declare_parameter<std::string>("frame_id", "arducam_stereo");
        this->declare_parameter<int>("width", 3840);
        this->declare_parameter<int>("height", 1080);
        this->declare_parameter<bool>("auto_exposure", true);
        this->declare_parameter<int>("exposure_value", 5);
        this->declare_parameter<bool>("auto_white_balance", true);
        this->declare_parameter<bool>("auto_wb_compensation", true);
        this->declare_parameter<int>("red_gain", 100);
        this->declare_parameter<int>("blue_gain", 100);

        this->get_parameter("frame_id", _frame_id);
        this->get_parameter("width", _width);
        this->get_parameter("height", _height);
        this->get_parameter("auto_exposure", _auto_exposure);
        this->get_parameter("exposure_value", _exposure_value);
        this->get_parameter("auto_white_balance", _auto_white_balance);
        this->get_parameter("auto_wb_compensation", _auto_wb_compensation);
        this->get_parameter("red_gain", _red_gain);
        this->get_parameter("blue_gain", _blue_gain);

        publisher_left = this->create_publisher<custom_interfaces::msg::H264Image>(
            _frame_id + "/h264_image/right", 10);
        publisher_right = this->create_publisher<custom_interfaces::msg::H264Image>(
            _frame_id + "/h264_image/left", 10);

        this->init_cam();
        this->init_decoder();
        this->init_filter(this->filter_descr);

        this->start_cam_thread();
        this->publish_images();
    }

  private:
    void init_cam() {
        int res;
        RCLCPP_INFO(this->get_logger(), "Opening camera...");
        res = arducam_init_camera(&camera_instance);
        if (res) {
            RCLCPP_INFO(this->get_logger(), "init camera status = %d", res);
            throw std::runtime_error("Could not init camera");
        }

        // Print modes and resolutions
        struct format support_fmt;
        int index = 0;
        char fourcc[5];
        fourcc[4] = '\0';
        while (!arducam_get_support_formats(camera_instance, &support_fmt, index++)) {
            strncpy(fourcc, (char*)&support_fmt.pixelformat, 4);
            RCLCPP_INFO(this->get_logger(),
                        "mode: %d, width: %d, height: %d, pixelformat: %s, desc: %s",
                        support_fmt.mode, support_fmt.width, support_fmt.height, fourcc,
                        support_fmt.description);
        }
        index = 0;
        struct camera_ctrl support_cam_ctrl;
        while (!arducam_get_support_controls(camera_instance, &support_cam_ctrl, index++)) {
            int value = 0;
            if (arducam_get_control(camera_instance, support_cam_ctrl.id, &value)) {
                RCLCPP_INFO(this->get_logger(), "Get ctrl %s fail.", support_cam_ctrl.desc);
            }
            RCLCPP_INFO(
                this->get_logger(),
                "index: %d, CID: 0x%08X, desc: %s, min: %d, max: %d, default: %d, current: %d",
                index - 1, support_cam_ctrl.id, support_cam_ctrl.desc, support_cam_ctrl.min_value,
                support_cam_ctrl.max_value, support_cam_ctrl.default_value, value);
        }

        // set resolution
        RCLCPP_INFO(this->get_logger(), "Setting resolution...");
        res = arducam_set_resolution(camera_instance, &_width, &_height);
        if (res) {
            RCLCPP_ERROR(this->get_logger(), "set resolution status = %d", res);
            throw std::runtime_error("Could not set resolution");
        } else {
            RCLCPP_INFO(this->get_logger(), "current resolution is %dx%d", _width, _height);
        }

        // set exposure auto or manual
        if (_auto_exposure) {
            RCLCPP_INFO(this->get_logger(), "Enabling Auto Exposure...");
            arducam_software_auto_exposure(camera_instance, 1);
        } else {
            if (arducam_set_control(camera_instance, V4L2_CID_EXPOSURE,
                                    (int)(_exposure_value * 0xFFFF / 200.0))) {
                RCLCPP_INFO(this->get_logger(),
                            "Failed to set exposure, the camera may not support this control.");
            }
        }

        // set white balance
        if (_auto_white_balance) {
            RCLCPP_INFO(this->get_logger(), "Enable Auto White Balance...");
            if (arducam_software_auto_white_balance(camera_instance, 1)) {
                RCLCPP_INFO(this->get_logger(), "Automatic white balance not supported");
            }
        }

        // set red and blue gain
        if (_auto_wb_compensation) {
            RCLCPP_INFO(this->get_logger(), "Enable Auto White Balance Compensation...");
            arducam_manual_set_awb_compensation(_red_gain, _blue_gain);
        }
    }

    void init_filter(const char* filters_descr) {
        RCLCPP_INFO(this->get_logger(), "Init Filter");
        char args[512];
        int ret = 0;
        const AVFilter* buffersrc = avfilter_get_by_name("buffer");
        const AVFilter* buffersink = avfilter_get_by_name("buffersink");
        AVFilterInOut* outputs = avfilter_inout_alloc();
        AVFilterInOut* inputs = avfilter_inout_alloc();
        // AVRational time_base = fmt_ctx->streams[video_stream_index]->time_base;
        enum AVPixelFormat pix_fmts[] = {AV_PIX_FMT_YUV420P, AV_PIX_FMT_NONE};

        filter_graph = avfilter_graph_alloc();
        if (!outputs || !inputs || !filter_graph) {
            RCLCPP_ERROR(this->get_logger(), "Could not start init frame");
            avfilter_inout_free(&inputs);
            avfilter_inout_free(&outputs);
        }

            /* buffer video source: the decoded frames from the decoder will be inserted here. */
        snprintf(args, sizeof(args),
            "video_size=%dx%d:pix_fmt=%d:time_base=%d/%d:pixel_aspect=%d/%d",
            p_frame_right->width,  p_frame_right->height, p_codec_context_right->pix_fmt,
            p_codec_context_right->time_base.num, p_codec_context_right->time_base.den,
            p_codec_context_right->sample_aspect_ratio.num, p_codec_context_right->sample_aspect_ratio.den);

        ret =
            avfilter_graph_create_filter(&buffersrc_ctx, buffersrc, "in", args, NULL, filter_graph);
        if (ret < 0) {
            RCLCPP_ERROR(this->get_logger(), "Cannot create buffer source");
            avfilter_inout_free(&inputs);
            avfilter_inout_free(&outputs);
        }

        /* buffer video sink: to terminate the filter chain. */
        ret = avfilter_graph_create_filter(&buffersink_ctx, buffersink, "out", NULL, NULL,
                                           filter_graph);
        if (ret < 0) {
            RCLCPP_ERROR(this->get_logger(), "Cannot create buffer sink");
            avfilter_inout_free(&inputs);
            avfilter_inout_free(&outputs);
        }

        ret = av_opt_set_int_list(buffersink_ctx, "pix_fmts", pix_fmts, AV_PIX_FMT_NONE,
                                  AV_OPT_SEARCH_CHILDREN);
        if (ret < 0) {
            RCLCPP_ERROR(this->get_logger(), "Cannot set output pixel format");
            avfilter_inout_free(&inputs);
            avfilter_inout_free(&outputs);
        }

        outputs->name = av_strdup("in");
        outputs->filter_ctx = buffersrc_ctx;
        outputs->pad_idx = 0;
        outputs->next = NULL;

        inputs->name = av_strdup("out");
        inputs->filter_ctx = buffersink_ctx;
        inputs->pad_idx = 0;
        inputs->next = NULL;

        if ((ret = avfilter_graph_parse_ptr(filter_graph, filters_descr, &inputs, &outputs, NULL)) <
            0) {
            avfilter_inout_free(&inputs);
            avfilter_inout_free(&outputs);
        }
        if ((ret = avfilter_graph_config(filter_graph, NULL)) < 0) {
            avfilter_inout_free(&inputs);
            avfilter_inout_free(&outputs);
        }
    }

    void init_decoder() {
        RCLCPP_INFO(this->get_logger(), "Init Decoder");
        av_register_all();
        avdevice_register_all();
        av_log_set_level(AV_LOG_INFO);

        av_init_packet(&packet_left);
        av_init_packet(&packet_right);

        // hardware encoder for 264 format
        p_codec_left = avcodec_find_encoder_by_name("h264_omx");
        if (!p_codec_left) {
            RCLCPP_ERROR(this->get_logger(), "Could not find ffmpeg h264 codec");
            throw std::runtime_error("Could not find ffmpeg h264 codec");
        }

        p_codec_context_left = avcodec_alloc_context3(p_codec_left);
        if (!p_codec_context_left) {
            RCLCPP_ERROR(this->get_logger(), "Could not allocate ffmpeg h264 codec");
            throw std::runtime_error("Could not open ffmpeg h264 codec");
        }

        // configer encoder
        p_codec_context_left->time_base = (AVRational){1, 1};
        p_codec_context_left->framerate = (AVRational){1, 1};
        p_codec_context_left->pix_fmt = AV_PIX_FMT_YUV420P;
        p_codec_context_left->width = _width / 2;
        p_codec_context_left->height = _height;
        p_codec_context_left->profile = FF_PROFILE_H264_HIGH; // FF_PROFILE_H264_STEREO_HIGH

        if (p_codec_left->id == AV_CODEC_ID_H264) {
            av_opt_set(p_codec_context_left->priv_data, "crf", "17", 0);
        }

        if (avcodec_open2(p_codec_context_left, p_codec_left, nullptr) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Could not open ffmpeg h264 codec");
            throw std::runtime_error("Could not open ffmpeg h264 codec");
        }

        // hardware encoder for 264 format
        p_codec_right = avcodec_find_encoder_by_name("h264_omx");
        if (!p_codec_right) {
            RCLCPP_ERROR(this->get_logger(), "Could not find ffmpeg h264 codec");
            throw std::runtime_error("Could not find ffmpeg h264 codec");
        }

        p_codec_context_right = avcodec_alloc_context3(p_codec_right);
        if (!p_codec_context_right) {
            RCLCPP_ERROR(this->get_logger(), "Could not allocate ffmpeg h264 codec");
            throw std::runtime_error("Could not open ffmpeg h264 codec");
        }

        // configer encoder
        p_codec_context_right->time_base = (AVRational){1, 1};
        p_codec_context_right->framerate = (AVRational){1, 1};
        p_codec_context_right->pix_fmt = AV_PIX_FMT_YUV420P;
        p_codec_context_right->width = _width / 2;
        p_codec_context_right->height = _height;
        p_codec_context_right->profile = FF_PROFILE_H264_HIGH; // FF_PROFILE_H264_STEREO_HIGH

        if (p_codec_right->id == AV_CODEC_ID_H264) {
            av_opt_set(p_codec_context_right->priv_data, "crf", "17", 0);
        }

        if (avcodec_open2(p_codec_context_right, p_codec_right, nullptr) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Could not open ffmpeg h264 codec");
            throw std::runtime_error("Could not open ffmpeg h264 codec");
        }

        // configer frame for encoder
        p_frame_left = av_frame_alloc();
        p_frame_left->format = p_codec_context_left->pix_fmt;
        p_frame_left->width = _width;
        p_frame_left->height = _height;

        if (av_frame_get_buffer(p_frame_left, 0) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Could not allocate the left video frame data");
            throw std::runtime_error("Could not allocate the left video frame data");
        }

        // configer frame for encoder
        p_frame_right = av_frame_alloc();
        p_frame_right->format = p_codec_context_right->pix_fmt;
        p_frame_right->width = _width;
        p_frame_right->height = _height;

        if (av_frame_get_buffer(p_frame_right, 0) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Could not allocate the right video frame data");
            throw std::runtime_error("Could not allocate the right video frame data");
        }
    }

    void start_cam_thread() {
        cam_thread_ = std::thread([this]() {
            RCLCPP_INFO(this->get_logger(), "Start cam thread");

            while (rclcpp::ok() && !stop_signal_) {
                // get image from cam
                IMAGE_FORMAT fmt = {IMAGE_ENCODING_I420, 50};
                BUFFER* buffer = arducam_capture(camera_instance, &fmt, 3000);
                if (!buffer)
                    continue;

                std::unique_lock<std::mutex> lk(mutex_);
                buffer_queue.push(buffer);
                count_++;

                lk.unlock();
                con_v.notify_all();
                /* sleep(1); */
            }

            // close camera instance
            int res;
            RCLCPP_INFO(this->get_logger(), "Close camera...");
            res = arducam_close_camera(camera_instance);
            if (res)
                RCLCPP_INFO(this->get_logger(), "close camera status = %d", res);
        });
    }

    void publish_images() {
        RCLCPP_INFO(this->get_logger(), "Start publish images");

        int res;
        bool flag_left;
        bool flag_right;
        filt_frame = av_frame_alloc();
        int _height_aligned = VCOS_ALIGN_UP(_height, 16);
        while (rclcpp::ok() && !stop_signal_) {

            std::unique_lock<std::mutex> lk(mutex_);
            // wait till buffer contains an image
            con_v.wait(lk, [this] { return count_ > 0; });

            if (count_ > 10) {
                for (int i = 0; i < 10; ++i) {
                    buf = buffer_queue.front();
                    buffer_queue.pop();
                    arducam_release_buffer(buf);
                    count_--;
                }
            }
            buf = buffer_queue.front();
            buffer_queue.pop();
            count_--;

            lk.unlock();

            custom_interfaces::msg::H264Image h264_msg_left;
            h264_msg_left.header.frame_id = "h264_left";
            h264_msg_left.seq = _seq;

            custom_interfaces::msg::H264Image h264_msg_right;
            h264_msg_right.header.frame_id = "h264_right";
            h264_msg_right.seq = _seq;

            // write cam image in frame
            res = av_image_fill_arrays(
                p_frame_left->data, p_frame_left->linesize,
                const_cast<uint8_t*>(reinterpret_cast<uint8_t const*>(buf->data)),
                AV_PIX_FMT_YUV420P, _width, _height_aligned, 16);
            if (res < 0) {
                RCLCPP_ERROR(this->get_logger(), "Could not fill image");
            }

            // write cam image in frame
            res = av_image_fill_arrays(
                p_frame_right->data, p_frame_right->linesize,
                const_cast<uint8_t*>(reinterpret_cast<uint8_t const*>(buf->data)),
                AV_PIX_FMT_YUV420P, _width, _height_aligned, 16);
            if (res < 0) {
                RCLCPP_ERROR(this->get_logger(), "Could not fill image");
            }

            // set image count for encoder
            p_frame_left->pts = _seq;
            p_frame_right->pts = _seq;

            /* push frame into the filtergraph */
            if (av_buffersrc_add_frame_flags(buffersrc_ctx, p_frame_right,
                                             AV_BUFFERSRC_FLAG_KEEP_REF) < 0) {
                av_log(NULL, AV_LOG_ERROR, "Error while feeding the filtergraph\n");
                break;
            }

            /* pull filtered frames from the filtergraph */
            res = av_buffersink_get_frame(buffersink_ctx, filt_frame);
            if (res == AVERROR(EAGAIN) || res == AVERROR_EOF) {
                RCLCPP_ERROR(this->get_logger(), "Could not get filtert frame");
            }
            if (res < 0) {
                RCLCPP_ERROR(this->get_logger(), "Filtergraph error");
                throw std::runtime_error("Filtergraph error");
            }

            // Send packet to encoder
            if (avcodec_send_frame(p_codec_context_right, filt_frame) < 0) {
                RCLCPP_ERROR(this->get_logger(), "Could not send packet");
            }
            // Get encoded frame
            if (avcodec_receive_packet(p_codec_context_right, &packet_right) < 0) {
                RCLCPP_ERROR(this->get_logger(), "Could not receive frame");
                flag_right = false;
            } else {
                flag_right = true;
            }
            // Send packet to encoder
            if (avcodec_send_frame(p_codec_context_left, p_frame_left) < 0) {
                RCLCPP_ERROR(this->get_logger(), "Could not send packet");
            }
            // Get encoded frame
            if (avcodec_receive_packet(p_codec_context_left, &packet_left) < 0) {
                RCLCPP_ERROR(this->get_logger(), "Could not receive frame");
                flag_left = false;
            } else {
                flag_left = true;
            }

            if (flag_left) {
                h264_msg_left.data.insert(h264_msg_left.data.begin(), &packet_left.data[0],
                                          &packet_left.data[packet_left.size]);
                h264_msg_left.header.stamp = this->now();

                if (publisher_left->get_subscription_count() > 0) {
                    publisher_left->publish(h264_msg_left);
                }
            }

            if (flag_right) {
                h264_msg_right.data.insert(h264_msg_right.data.begin(), &packet_right.data[0],
                                           &packet_right.data[packet_right.size]);
                h264_msg_right.header.stamp = this->now();

                if (publisher_right->get_subscription_count() > 0) {
                    publisher_right->publish(h264_msg_right);
                }
            }

            _seq++;
            arducam_release_buffer(buf);
        }

        avfilter_graph_free(&filter_graph);
        avcodec_free_context(&p_codec_context_left);
        avcodec_free_context(&p_codec_context_right);
        av_frame_free(&p_frame_left);
        av_frame_free(&p_frame_right);
        av_frame_free(&filt_frame);

        if (cam_thread_.joinable()) {
            stop_signal_ = true;
            cam_thread_.join();
        }
    }

    std::queue<BUFFER*> buffer_queue;
    std::condition_variable con_v;
    std::mutex mutex_;
    int count_ = 0;
    bool stop_signal_ = false;

    std::string _frame_id;
    int _width;
    int _height;
    int _width_aligned;
    int _height_aligned;
    int _align_width_down;
    int _align_width_up;
    int _align_height_down;
    int _align_height_up;
    int _seq = 0;
    // belichtungszeit auto empfohlen
    bool _auto_exposure;
    // manuelle belichtungszeit => value/1000 (100us)
    int _exposure_value;
    bool _auto_white_balance;
    bool _auto_wb_compensation;
    int _red_gain;
    int _blue_gain;
    std::thread cam_thread_;
    BUFFER* buf;
    BUFFER* buf_copy;
    std::vector<uint8_t> vec;

    int consecutive_receive_failures_;
    AVPacket packet_left;
    AVCodec* p_codec_left{nullptr};
    AVCodecContext* p_codec_context_left{nullptr};
    AVFrame* p_frame_left{nullptr};

    AVPacket packet_right;
    AVCodec* p_codec_right{nullptr};
    AVCodecContext* p_codec_context_right{nullptr};
    AVFrame* p_frame_right{nullptr};

    AVFilterContext* buffersink_ctx;
    AVFilterContext* buffersrc_ctx;
    AVFilterGraph* filter_graph;
    AVFrame* filt_frame;
    const char* filter_descr = "crop=1/2*in_w:in_h:1/2*in_w:0";

    CAMERA_INSTANCE camera_instance;
    rclcpp::Publisher<custom_interfaces::msg::H264Image>::SharedPtr publisher_left;
    rclcpp::Publisher<custom_interfaces::msg::H264Image>::SharedPtr publisher_right;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArducamStereoNode>());
    rclcpp::shutdown();
    return 0;
}
