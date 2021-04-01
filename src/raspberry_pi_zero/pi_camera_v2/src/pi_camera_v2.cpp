#include <chrono>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/time.h>
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
            1000ms, std::bind(&PiCamera::timer_callback, this));

        // open camera file
        fd_ = open(DEVICE, O_RDWR);
        if (fd_ < 0) {
            perror(DEVICE);
        }
        cropcaps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        // init device
        fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        // init mmap
        reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        this->print_caps();
        this->print_available_formats();

        this->init_device();
        this->init_mmap();
    }

    ~PiCamera() { close(fd_); }

   private:
    int init_device() {
        fmtdesc.index = fmt_index_;
        ret_ = ioctl(fd_, VIDIOC_ENUM_FMT, &fmtdesc);
        if (ret_ == -1) {
            perror("Init Device Format:");
            return errno;
        }
        printf("\nUsing format: %s\n", fmtdesc.description);

        fmt.fmt.pix.width = width_;
        fmt.fmt.pix.height = height_;
        fmt.fmt.pix.pixelformat = fmtdesc.pixelformat;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;

        ret_ = ioctl(fd_, VIDIOC_S_FMT, &fmt);
        if (ret_ == -1) {
            perror("Set Format:");
            return -1;
        }
        this->print_used_format();

        return 0;
    }

    int init_mmap() {
        reqbuf.memory = V4L2_MEMORY_MMAP;
        reqbuf.count = 5;
        ret_ = ioctl(fd_, VIDIOC_REQBUFS, &reqbuf);
        if (ret_ == -1) {
            perror("VIDIOC_REQBUFS:");
            return errno;
        }
        if (reqbuf.count < 2) {
            printf("Not enough buffer memory\n");
            return -1;
        }
        buffers = (struct buf*)calloc(reqbuf.count, sizeof(*buffers));
        assert(buffers != NULL);

        num_buffers_ = reqbuf.count;
        printf("MMAP Buffer number: %d", num_buffers_);
        for (unsigned int i = 0; i < reqbuf.count; i++) {
            memset(&buffer, 0, sizeof(buffer));
            buffer.type = reqbuf.type;
            buffer.memory = V4L2_MEMORY_MMAP;
            buffer.index = i;

            ret_ = ioctl(fd_, VIDIOC_QUERYBUF, &buffer);
            if (ret_ == -1) {
                perror("VIDIOC_QUERYBUF:");
                return errno;
            }

            buffers[i].length = buffer.length;
            buffers[i].start = mmap(NULL, buffer.length, PROT_READ | PROT_WRITE,
                                    MAP_SHARED, fd_, buffer.m.offset);

            if (MAP_FAILED == buffers[i].start) {
                perror("mmap");
                return errno;
            }
        }
        return 0;
    }

    int start_capturing() {
        enum v4l2_buf_type type;

        for (unsigned int i = 0; i < num_buffers_; i++) {
            // Note that we set bytesused = 0, which will set it to the buffer
            // length
            memset(&buffer, 0, sizeof(buffer));
            buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buffer.memory = V4L2_MEMORY_MMAP;
            buffer.index = i;

            // Enqueue the buffer with VIDIOC_QBUF
            ret_ = ioctl(fd_, VIDIOC_QBUF, &buffer);
            if (ret_ == -1) {
                perror("VIDIOC_QBUF");
                return errno;
            }
        }

        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        ret_ = ioctl(fd_, VIDIOC_STREAMON, &type);
        if (ret_ == -1) {
            perror("VIDIOC_STREAMON");
            return errno;
        }
        return 0;
    }

    void print_used_format() {
        char format_code[5];
        strncpy(format_code, (char*)&fmt.fmt.pix.pixelformat, 5);
        printf(
            "Set format:\n"
            " Width: %d\n"
            " Height: %d\n"
            " Pixel format: %s\n"
            " Field: %d\n\n",
            fmt.fmt.pix.width, fmt.fmt.pix.height, format_code,
            fmt.fmt.pix.field);
    }

    int print_caps() {
        ret_ = ioctl(fd_, VIDIOC_QUERYCAP, &caps);
        if (ret_ == -1) {
            perror("Querying device capabilities");
            return errno;
        }

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
        return 0;
    }

    int print_crops() {
        ret_ = ioctl(fd_, VIDIOC_CROPCAP, &cropcaps);
        if (ret_ == -1) {
            perror("Querying device crop capabilities");
            return errno;
        }
        RCLCPP_INFO(this->get_logger(),
                    "Camera Cropping:\n"
                    "  Bounds: %dx%d+%d+%d\n"
                    "  Default: %dx%d+%d+%d\n"
                    "  Aspect: %d/%d\n",
                    cropcaps.bounds.width, cropcaps.bounds.height,
                    cropcaps.bounds.left, cropcaps.bounds.top,
                    cropcaps.defrect.width, cropcaps.defrect.height,
                    cropcaps.defrect.left, cropcaps.defrect.top,
                    cropcaps.pixelaspect.numerator,
                    cropcaps.pixelaspect.denominator);
        return 0;
    }

    int print_available_formats() {
        // These will correspond to the flags V4L2_FMT_FLAG_COMPRESSED and
        // V4L2_FMT_FLAG_EMULATED
        char c, e;

        while (0 == ioctl(fd_, VIDIOC_ENUM_FMT, &fmtdesc)) {
            c = fmtdesc.flags & 1 ? 'C' : ' ';
            e = fmtdesc.flags & 2 ? 'E' : ' ';

            printf("%2d %c%c %s\n", fmtdesc.index, c, e, fmtdesc.description);
            fmtdesc.index++;
        }

        return 0;
    }

    void timer_callback() {
        auto message = std_msgs::msg::String();
        message.data = "Hello";
        publisher_->publish(message);
    }

    struct buf {
        void* start;
        size_t length;
    };
    struct buf* buffers = NULL;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    int fd_;
    int ret_;
    unsigned int num_buffers_;
    int width_ = 1920;
    int height_ = 1080;
    int fmt_index_ = 4;
    struct v4l2_capability caps = {0};
    struct v4l2_cropcap cropcaps = {0};
    // init device
    struct v4l2_fmtdesc fmtdesc = {0};
    struct v4l2_format fmt = {0};
    // init mmap
    struct v4l2_requestbuffers reqbuf = {0};
    struct v4l2_buffer buffer;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PiCamera>());
    rclcpp::shutdown();

    return 0;
}
