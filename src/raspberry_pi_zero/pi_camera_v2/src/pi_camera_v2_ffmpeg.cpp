#include <memory>
#include <string>
#include <utility>
#include <vector>

extern "C" {
#include "libavdevice/avdevice.h"
#include "libavutil/imgutils.h"
}
#include <opencv2/opencv.hpp>

#include "custom_interfaces/msg/h264_image.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/node.hpp>

class PiCamera : public rclcpp::Node {
  public:
    PiCamera() : Node("pi_cam") {
        publisher_ =
            this->create_publisher<custom_interfaces::msg::H264Image>("pi_cam/h264_image", 10);

        this->declare_parameter<std::string>("widthxheight", "1920x1080");
        this->declare_parameter<std::string>("fps", "30");
        this->get_parameter("widthxheight", size_);
        this->get_parameter("fps", fps_);

        // Initialize libavdevice and register all the input and output devices.
        avdevice_register_all();
        av_log_set_level(AV_LOG_WARNING);

        // Device drivers appear as formats in ffmpeg
        // Find the v4l driver
        input_format_ = av_find_input_format("video4linux2");
        if (!input_format_) {
            RCLCPP_ERROR(get_logger(), "Could not find the v4l driver");
            throw std::runtime_error("Could not find the v4l driver");
        }

        // Allocate a format context
        format_context_ = avformat_alloc_context();
        if (!format_context_) {
            RCLCPP_ERROR(get_logger(), "Could not allocate a format context");
            throw std::runtime_error("Could not allocate a format context");
        }

        // Set format options, this will allocate an AVDictionary
        AVDictionary* format_options = nullptr;
        av_dict_set(&format_options, "input_format", "h264", 0);
        av_dict_set(&format_options, "framerate", fps_.c_str(), 0);
        av_dict_set(&format_options, "video_size", size_.c_str(), 0);

        // Open 4vl device, pass ownership of format_options
        if (avformat_open_input(&format_context_, fd_.c_str(), input_format_, &format_options) <
            0) {
            RCLCPP_ERROR_STREAM(get_logger(), "Could not open the v4l device: " << fd_);
            throw std::runtime_error("Could not open the v4l device");
        }

        this->start_stream();
    }

    void start_stream() {
        while (rclcpp::ok()) {
            custom_interfaces::msg::H264Image h264_msg;
            h264_msg.header.frame_id = "pi_cam";
            h264_msg.seq = nFrames_++;

            if (nFrames_ % 10 == 0) {
                const int N = 10;
                int64 t1 = cv::getTickCount();
                std::cout << "Frames captured: " << cv::format("%5lld", (long long int)nFrames_)
                          << "    Average FPS: "
                          << cv::format("%9.1f", (double)cv::getTickFrequency() * N / (t1 - t0))
                          << "    Average time per frame: "
                          << cv::format("%9.2f ms",
                                        (double)(t1 - t0) * 1000.0f / (N * cv::getTickFrequency()))
                          << "    Average processing time: "
                          << cv::format("%9.2f ms", (double)(processingTime)*1000.0f /
                                                        (N * cv::getTickFrequency()))
                          << std::endl;
                t0 = t1;
                processingTime = 0;
            }

            // Block until a frame is ready
            AVPacket packet;
            if (av_read_frame(format_context_, &packet) < 0) {
                RCLCPP_INFO(get_logger(), "EOS");
                break;
            }

            int64 tp0 = cv::getTickCount();
            // Copy to the ROS message and free the packet
            if (publisher_->get_subscription_count() > 0) {
                h264_msg.data.insert(h264_msg.data.begin(), &packet.data[0],
                                     &packet.data[packet.size]);
                h264_msg.header.stamp = this->now();
                publisher_->publish(h264_msg);
            }
            av_packet_unref(&packet);

            processingTime += cv::getTickCount() - tp0;
        }
        // Close v4l device
        avformat_close_input(&format_context_);
    }

    ~PiCamera() {
        RCLCPP_INFO(this->get_logger(), "Deconstructor PiCamera");
        avformat_free_context(format_context_);
    }

  private:
    int nFrames_;
    int64 t0 = 0;
    int64 processingTime = 0;
    std::string size_;
    std::string fps_;
    std::string fd_ = "/dev/video0";
    AVFormatContext* format_context_{nullptr};
    AVInputFormat* input_format_{nullptr};
    rclcpp::Publisher<custom_interfaces::msg::H264Image>::SharedPtr publisher_;
};

int main(int argc, char** argv) {
    setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PiCamera>());
    rclcpp::shutdown();
    return 0;
}
