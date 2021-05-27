#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <fcntl.h> // open filedescriptor
#include <unistd.h> // close filedescriptor

extern "C" {
#include "libavdevice/avdevice.h"
#include "libavutil/imgutils.h"
}

#include "custom_interfaces/msg/h264_image.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/node.hpp>

static const char DEVICE[] = "/dev/video0";

class PiCamera : public rclcpp::Node {
  public:
    PiCamera() : Node("pi_cam"), nFrames_(0) {
        this->declare_parameter<std::string>("size", "800x600");
        this->declare_parameter<std::string>("fps", "30");
        this->declare_parameter<std::string>("frame_id", "pi_cam");
        this->declare_parameter<int>("rotation", 0);

        this->get_parameter("size", size_);
        this->get_parameter("fps", fps_);
        this->get_parameter("frame_id", frame_id_);
        this->get_parameter("rotation", rotation_);

        RCLCPP_INFO_STREAM(get_logger(), "Parameter input_fd: " << fd_);
        RCLCPP_INFO_STREAM(get_logger(), "Parameter fps: " << fps_);
        RCLCPP_INFO_STREAM(get_logger(), "Parameter size: " << size_);
        RCLCPP_INFO_STREAM(get_logger(), "Parameter frame_id: " << frame_id_);

        publisher_ = this->create_publisher<custom_interfaces::msg::H264Image>(
            frame_id_ + "/h264_image", 10);

        // TODO: make control settings permanent
        /* this->set_v4l2_controls(); */
        // Initialize libavdevice and register all the input and output devices.
        av_register_all();
        avdevice_register_all();
        av_log_set_level(AV_LOG_INFO);

        // Device drivers appear as formats in ffmpeg
        // Find the v4l driver
        AVInputFormat* input_format = av_find_input_format("video4linux2");
        if (!input_format) {
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
    }

    void start_stream() {
        while (rclcpp::ok()) {
            custom_interfaces::msg::H264Image h264_msg;
            h264_msg.header.frame_id = frame_id_;
            h264_msg.seq = nFrames_++;

            // Block until a frame is ready
            AVPacket packet;
            if (av_read_frame(format_context_, &packet) < 0) {
                RCLCPP_INFO(get_logger(), "EOS");
                break;
            }

            // Copy to the ROS message and free the packet
            if (publisher_->get_subscription_count() > 0) {
                h264_msg.data.insert(h264_msg.data.begin(), &packet.data[0],
                                     &packet.data[packet.size]);
                h264_msg.header.stamp = this->now();
                publisher_->publish(h264_msg);
            }
            av_packet_unref(&packet);
        }
        // Close v4l device
        RCLCPP_INFO(this->get_logger(), "Close v4l2 device");
        avformat_close_input(&format_context_);
        avformat_free_context(format_context_);
    }

    ~PiCamera() { RCLCPP_INFO(this->get_logger(), "Deconstructor PiCamera"); }

  private:
    void set_v4l2_controls(){
        // open camera file
        int fd;
        fd = open(DEVICE, O_RDWR);
        if (fd < 0) {
            RCLCPP_ERROR_STREAM(get_logger(), "Could not open the v4l2 device for control settings: " << fd_);
            throw std::runtime_error("Could not open the v4l2 device for control settings");
        }

        memset(&control, 0, sizeof (control));
        control.id = V4L2_CID_ROTATE;
        control.value = rotation_;

        if (-1 == ioctl(fd, VIDIOC_S_CTRL, &control)) {
            RCLCPP_ERROR_STREAM(get_logger(), "Could not set rotation control: " << fd_);
            throw std::runtime_error("Could not set rotation control");
        }
        RCLCPP_INFO(this->get_logger(), "set rotation: %u", rotation_);

        close(fd);
    }

    int nFrames_;
    int rotation_;
    std::string size_;
    std::string fps_;
    std::string frame_id_;
    std::string fd_ = "/dev/video0";
    struct v4l2_control control;
    AVFormatContext* format_context_{nullptr};
    AVInputFormat* input_format_{nullptr};
    rclcpp::Publisher<custom_interfaces::msg::H264Image>::SharedPtr publisher_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<PiCamera> cam_node = std::make_shared<PiCamera>();
    cam_node->start_stream();
    rclcpp::shutdown();
    return 0;
}
