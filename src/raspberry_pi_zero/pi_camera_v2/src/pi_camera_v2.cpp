#include <chrono>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>

#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

static const char DEVICE[] = "/dev/video0";

class PiCamera : public rclcpp::Node {
   public:
    PiCamera() : Node("pi_camera") {
        publisher_ =
            this->create_publisher<std_msgs::msg::String>("camera_caps", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&PiCamera::timer_callback, this));

        fd = open(DEVICE, O_RDWR);
        if (fd < 0) {
            perror(DEVICE);
        }
    }
    void close_devise() {
        RCLCPP_INFO(this->get_logger(), "close devise: %s\n", DEVICE);
        close(fd);
    }

   private:
    void timer_callback() {
        struct v4l2_capability caps = {0};

        ret = ioctl(fd, VIDIOC_QUERYCAP, &caps);
        if (ret == -1) {
            perror("Querying device capabilities");
        }

        auto message = std_msgs::msg::String();
        message.data = "Hello World";

        RCLCPP_INFO(this->get_logger(),
                    "Driver Caps:\n"
                    "  Driver: \"%s\"\n"
                    "  Card: \"%s\"\n"
                    "  Bus: \"%s\"\n"
                    "  Version: %u.%u.%u\n"
                    "  Capabilities: %08x\n",
                    caps.driver, caps.card, caps.bus_info,
                    (caps.version >> 16) & 0xFF, (caps.version >> 8) & 0xFF,
                    (caps.version) & 0XFF, caps.capabilities);

        publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    int fd;
    int ret;
};

int main(int argc, char** argv) {
    printf("hello world pi_camera_v2 package\n");

    rclcpp::init(argc, argv);

    auto cam_node = std::make_shared<PiCamera>();
    rclcpp::spin(cam_node);
    cam_node->close_devise();
    rclcpp::shutdown();
    return 0;
}
