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
}

#include "custom_interfaces/msg/h264_image.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using std::placeholders::_1;

class H264ToRawSubscriber : public rclcpp::Node {
  public:
    H264ToRawSubscriber()
        : Node("h264_raw_subscriber"), nFrames_(0), consecutive_receive_failures_(0) {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/pi_cam/raw", 10);
        subscription_ = this->create_subscription<custom_interfaces::msg::H264Image>(
            "/pi_cam/h264_image", 10, std::bind(&H264ToRawSubscriber::h264_callback, this, _1));

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
        RCLCPP_INFO(this->get_logger(), "data size: %u", h264_msg->data.size());
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

        RCLCPP_INFO(this->get_logger(), "size: %u x %u", p_frame_->width, p_frame_->height);
        auto image = sensor_msgs::msg::Image();
        image.width = p_frame_->width;
        image.height = p_frame_->height;
        image.step = 3 * p_frame_->width;
        image.encoding = sensor_msgs::image_encodings::BGR8;
        image.header = h264_msg->header;

        publisher_->publish(image);
    }

    int nFrames_;
    int consecutive_receive_failures_;
    AVPacket packet_;
    AVCodec* p_codec_{nullptr};
    AVCodecContext* p_codec_context_{nullptr};
    AVFrame* p_frame_{nullptr};
    rclcpp::Subscription<custom_interfaces::msg::H264Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<H264ToRawSubscriber>());
    rclcpp::shutdown();
    return 0;
}

