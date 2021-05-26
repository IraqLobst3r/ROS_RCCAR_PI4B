#include <memory>
#include <rclcpp/publisher_base.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <string>
#include <utility>
#include <vector>

extern "C" {
#include "libavdevice/avdevice.h"
#include "libavutil/imgutils.h"
#include "libswscale/swscale.h"
}

#include "custom_interfaces/msg/h264_image.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using std::placeholders::_1;

class H264ToRawSubscriber : public rclcpp::Node {
  public:
    H264ToRawSubscriber()
        : Node("h264_raw_subscriber"), nFrames_(0), consecutive_receive_failures_(0) {

        this->declare_parameter<std::string>("pub_topic", "/pi_cam/raw");
        this->declare_parameter<std::string>("sub_topic", "/pi_cam/h264_image");
        this->get_parameter("pub_topic", pub_topic_);
        this->get_parameter("sub_topic", sub_topic_);

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(pub_topic_, 10);
        subscription_ = this->create_subscription<custom_interfaces::msg::H264Image>(
            sub_topic_, 10, std::bind(&H264ToRawSubscriber::h264_callback, this, _1));

        av_init_packet(&packet_);
        av_log_set_level(AV_LOG_WARNING);

        p_codec_ = avcodec_find_decoder(AV_CODEC_ID_H264);
        if (!p_codec_) {
            RCLCPP_ERROR(this->get_logger(), "Could not find ffmpeg h264 codec");
            throw std::runtime_error("Could not find ffmpeg h264 codec");
        }

        p_codec_context_ = avcodec_alloc_context3(p_codec_);

        if (avcodec_open2(p_codec_context_, p_codec_, nullptr) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Could not open ffmpeg h264 codec");
            throw std::runtime_error("Could not open ffmpeg h264 codec");
        }

        p_frame_ = av_frame_alloc();

        RCLCPP_INFO(this->get_logger(), "Init Subscription");
    }

  private:
    void h264_callback(const custom_interfaces::msg::H264Image::SharedPtr h264_msg) {
        // Report on sequence problems
        if (nFrames_ < 0) {
            RCLCPP_INFO(this->get_logger(), "First message: %d", h264_msg->seq);
        } else {
            if (h264_msg->seq < nFrames_) {
                RCLCPP_INFO(this->get_logger(), "Old message: %d", h264_msg->seq);
            }
            if (h264_msg->seq == nFrames_) {
                RCLCPP_INFO(this->get_logger(), "Repeat message: %d", nFrames_);
            }
            if (h264_msg->seq > nFrames_ + 1) {
                RCLCPP_INFO(this->get_logger(), "Missing message(s): %d-%d", nFrames_ + 1,
                            h264_msg->seq - 1);
            }
        }
        nFrames_ = h264_msg->seq;

        packet_.size = h264_msg->data.size();
        packet_.data = const_cast<uint8_t*>(reinterpret_cast<uint8_t const*>(&h264_msg->data[0]));

        // Send packet to decoder
        if (avcodec_send_packet(p_codec_context_, &packet_) < 0) {
            RCLCPP_INFO(this->get_logger(), "Could not send packet");
            return;
        }

        // Get decoded frame
        // Failure to decode is common when first starting, avoid spamming the logs
        if (avcodec_receive_frame(p_codec_context_, p_frame_) < 0) {
            if (++consecutive_receive_failures_ % 30 == 0) {
                RCLCPP_INFO(this->get_logger(), "Could not receive %d frames",
                            consecutive_receive_failures_);
            }
            return;
        }

        auto image = sensor_msgs::msg::Image();
        image.width = p_frame_->width;
        image.height = p_frame_->height;
        image.step = 3 * p_frame_->width;
        image.encoding = sensor_msgs::image_encodings::BGR8;
        image.header = h264_msg->header;

        // Set / update sws context
        p_sws_context_ = sws_getCachedContext(
            p_sws_context_, p_frame_->width, p_frame_->height, AV_PIX_FMT_YUV420P, p_frame_->width,
            p_frame_->height, AV_PIX_FMT_BGR24, SWS_FAST_BILINEAR, nullptr, nullptr, nullptr);

        // Copy and convert from YUYV420P to BGR24
        image.data.resize(p_frame_->width * p_frame_->height * 3);
        int stride = 3 * p_frame_->width;
        uint8_t* destination = &image.data[0];
        sws_scale(p_sws_context_, (const uint8_t* const*)p_frame_->data, p_frame_->linesize, 0,
                  p_frame_->height, &destination, &stride);

        publisher_->publish(image);
    }

    int nFrames_;
    int consecutive_receive_failures_;
    std::string pub_topic_;
    std::string sub_topic_;
    AVPacket packet_;
    AVCodec* p_codec_{nullptr};
    AVCodecContext* p_codec_context_{nullptr};
    AVFrame* p_frame_{nullptr};
    SwsContext* p_sws_context_{nullptr};
    rclcpp::Subscription<custom_interfaces::msg::H264Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<H264ToRawSubscriber>());
    rclcpp::shutdown();
    return 0;
}

