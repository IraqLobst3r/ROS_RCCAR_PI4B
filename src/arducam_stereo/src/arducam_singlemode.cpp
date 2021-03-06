
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
}

#include "arducam_mipicamera.h"

#include "custom_interfaces/msg/h264_image.hpp"
#include "rclcpp/rclcpp.hpp"

#define VCOS_ALIGN_DOWN(p, n) (((ptrdiff_t)(p)) & ~((n)-1))
#define VCOS_ALIGN_UP(p, n) VCOS_ALIGN_DOWN((ptrdiff_t)(p) + (n)-1, (n))

std::mutex mutex_run;
bool run = true;
int width_aligned;
int height_aligned;

class ArducamStereoNode : public rclcpp::Node {
  public:
    ArducamStereoNode() : Node("arducam_stereo_node") {

        this->declare_parameter<std::string>("frame_id", "arducam_cam0");
        this->declare_parameter<int>("cam_interface", 0);
        this->declare_parameter<int>("width", 1920);
        this->declare_parameter<int>("height", 1080);
        this->declare_parameter<bool>("auto_exposure", true);
        this->declare_parameter<int>("exposure_value", 5);
        this->declare_parameter<bool>("auto_white_balance", true);
        this->declare_parameter<bool>("auto_wb_compensation", true);
        this->declare_parameter<int>("red_gain", 100);
        this->declare_parameter<int>("blue_gain", 100);

        this->get_parameter("frame_id", _frame_id);
        this->get_parameter("cam_interface", _cam_num);
        this->get_parameter("width", _width);
        this->get_parameter("height", _height);
        this->get_parameter("auto_exposure", _auto_exposure);
        this->get_parameter("exposure_value", _exposure_value);
        this->get_parameter("auto_white_balance", _auto_white_balance);
        this->get_parameter("auto_wb_compensation", _auto_wb_compensation);
        this->get_parameter("red_gain", _red_gain);
        this->get_parameter("blue_gain", _blue_gain);

        publisher_ = this->create_publisher<custom_interfaces::msg::H264Image>(
            _frame_id + "/h264_image", 10);

        struct camera_interface cam_interface = {
            .i2c_bus = 0,           // /dev/i2c-0  or /dev/i2c-1
            .camera_num = 0, // mipi interface num
            .sda_pins = {0, 28},
            .scl_pins = {1, 29},
            .shutdown_pins = {3, 31},
            .led_pins = {2, 30},
        };
        int res;
        RCLCPP_INFO(this->get_logger(), "Opening camera...");
        res = arducam_init_camera2(&camera_instance, cam_interface);
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

        av_register_all();
        avdevice_register_all();
        av_log_set_level(AV_LOG_INFO);

        av_init_packet(&packet_);

        // software encoder for 264 format
        /* p_codec_ = avcodec_find_encoder(AV_CODEC_ID_H264); */
        // hardware encoder for 264 format
        p_codec_ = avcodec_find_encoder_by_name("h264_omx");
        if (!p_codec_) {
            RCLCPP_ERROR(this->get_logger(), "Could not find ffmpeg h264 codec");
            throw std::runtime_error("Could not find ffmpeg h264 codec");
        }

        p_codec_context_ = avcodec_alloc_context3(p_codec_);
        if (!p_codec_context_) {
            RCLCPP_ERROR(this->get_logger(), "Could not allocate ffmpeg h264 codec");
            throw std::runtime_error("Could not open ffmpeg h264 codec");
        }

        // configer encoder
        p_codec_context_->time_base = (AVRational){1, 1};
        p_codec_context_->framerate = (AVRational){1, 1};
        p_codec_context_->pix_fmt = AV_PIX_FMT_YUV420P;
        p_codec_context_->width = _width;
        p_codec_context_->height = _height;
        p_codec_context_->profile = FF_PROFILE_H264_HIGH;

        if (p_codec_->id == AV_CODEC_ID_H264) {
            av_opt_set(p_codec_context_->priv_data, "crf", "17", 0);
        }

        if (avcodec_open2(p_codec_context_, p_codec_, nullptr) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Could not open ffmpeg h264 codec");
            throw std::runtime_error("Could not open ffmpeg h264 codec");
        }

        // configer frame for encoder
        p_frame_ = av_frame_alloc();
        p_frame_->format = p_codec_context_->pix_fmt;
        p_frame_->width = _width;
        p_frame_->height = _height;

        if (av_frame_get_buffer(p_frame_, 0) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Could not allocate the left video frame data");
            throw std::runtime_error("Could not allocate the left video frame data");
        }

        // crop image
        p_frame_->crop_left = 0;
        p_frame_->crop_top = 0;
        p_frame_->crop_right = 0;
        p_frame_->crop_bottom = 0;
        if (av_frame_apply_cropping(p_frame_, 0) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Could not crop left frame");
        }
        RCLCPP_INFO(this->get_logger(), "crop frame OK");

        start_cam_thread();
        publish_images();
    }

  private:
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
                sleep(1);
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

            custom_interfaces::msg::H264Image h264_msg_;
            h264_msg_.header.frame_id = "h264";
            h264_msg_.seq = _seq;

            // write cam image in frame
            res = av_image_fill_arrays(
                p_frame_->data, p_frame_->linesize,
                const_cast<uint8_t*>(reinterpret_cast<uint8_t const*>(buf->data)),
                AV_PIX_FMT_YUV420P, _width_aligned, _height_aligned, 32);
            if (res < 0) {
                RCLCPP_ERROR(this->get_logger(), "Could not fill image");
            }
            RCLCPP_INFO(this->get_logger(), "image to left frame OK");

            // set image count for encoder
            p_frame_->pts = _seq;

            // Send packet to encoder
            if (avcodec_send_frame(p_codec_context_, p_frame_) < 0) {
                RCLCPP_ERROR(this->get_logger(), "Could not send packet");
            }
            // Get encoded frame
            if (avcodec_receive_packet(p_codec_context_, &packet_) < 0) {
                RCLCPP_ERROR(this->get_logger(), "Could not receive frame");
            } else {
                h264_msg_.data.insert(h264_msg_.data.begin(), &packet_.data[0],
                                      &packet_.data[packet_.size]);
                h264_msg_.header.stamp = this->now();

                if (publisher_->get_subscription_count() > 0) {
                    publisher_->publish(h264_msg_);
                }
            }

            _seq++;
            arducam_release_buffer(buf);
        }

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
    int _cam_num;
    int _width;
    int _height;
    int _width_aligned;
    int _height_aligned;
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
    std::vector<uint8_t> vec;

    AVPacket packet_;
    AVCodec* p_codec_{nullptr};
    AVCodecContext* p_codec_context_{nullptr};
    AVFrame* p_frame_{nullptr};

    CAMERA_INSTANCE camera_instance;
    rclcpp::Publisher<custom_interfaces::msg::H264Image>::SharedPtr publisher_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArducamStereoNode>());
    rclcpp::shutdown();
    return 0;
}
