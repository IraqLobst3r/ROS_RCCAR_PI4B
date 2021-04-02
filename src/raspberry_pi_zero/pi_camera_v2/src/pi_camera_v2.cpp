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

#define GET_VAR_NAME(name) (#name)

using namespace std::chrono_literals;

static const char DEVICE[] = "/dev/video0";

class PiCamera : public rclcpp::Node {
   public:
    PiCamera() : Node("pi_camera") {
        publisher_ =
            this->create_publisher<std_msgs::msg::String>("camera_image", 10);
        // init device
        fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        // init mmap
        reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        reqbuf.memory = V4L2_MEMORY_MMAP;
        reqbuf.count = 5;
    }

    void init_device() {
        // open camera file
        fd_ = open(DEVICE, O_RDWR);
        if (fd_ < 0) {
            perror(DEVICE);
            throw;
        }

        // get format
        fmtdesc.index = fmt_index_;
        this->xioctl(fd_, VIDIOC_ENUM_FMT, &fmtdesc);

        fmt.fmt.pix.width = width_;
        fmt.fmt.pix.height = height_;
        fmt.fmt.pix.pixelformat = fmtdesc.pixelformat;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;

        this->xioctl(fd_, VIDIOC_S_FMT, &fmt);
        this->print_used_format();
        this->init_mmap();
    }

    void start_capturing() {
        enum v4l2_buf_type type;

        struct v4l2_buffer buffer;
        for (unsigned int i = 0; i < num_buffers_; i++) {
            // Note that we set bytesused = 0, which will set it to the buffer
            // length
            memset(&buffer, 0, sizeof(buffer));
            buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buffer.memory = V4L2_MEMORY_MMAP;
            buffer.index = i;

            // Enqueue the buffer with VIDIOC_QBUF
            this->xioctl(fd_, VIDIOC_QBUF, &buffer);
        }

        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        this->xioctl(fd_, VIDIOC_STREAMON, &type);

        this->loop();
    }

    void print_caps() {
        this->xioctl(fd_, VIDIOC_QUERYCAP, &caps);
        printf(
            "Driver Caps:\n"
            "  Driver: \"%s\"\n"
            "  Card: \"%s\"\n"
            "  Bus: \"%s\"\n"
            "  Version: %u.%u.%u\n"
            "  Capabilities: %08x\n",
            caps.driver, caps.card, caps.bus_info, (caps.version >> 16) & 0xFF,
            (caps.version >> 8) & 0xFF, (caps.version) & 0XFF,
            caps.capabilities);
    }

    void print_available_formats() {
        // These will correspond to the flags V4L2_FMT_FLAG_COMPRESSED and
        // V4L2_FMT_FLAG_EMULATED
        char c, e;

        while (0 == ioctl(fd_, VIDIOC_ENUM_FMT, &fmtdesc)) {
            c = fmtdesc.flags & 1 ? 'C' : ' ';
            e = fmtdesc.flags & 2 ? 'E' : ' ';

            printf("%2d %c%c %s\n", fmtdesc.index, c, e, fmtdesc.description);
            fmtdesc.index++;
        }
    }

    ~PiCamera() {
        printf("Deconstructor PiCamera");
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        this->xioctl(fd_, VIDIOC_STREAMOFF, &type);
        // Cleanup
        for (unsigned int i = 0; i < reqbuf.count; i++)
            munmap(buffers[i].start, buffers[i].length);
        free(buffers);
        close(fd_);
    }

   private:
    void init_mmap() {
        this->xioctl(fd_, VIDIOC_REQBUFS, &reqbuf);
        if (reqbuf.count < 2) {
            printf("Not enough buffer memory\n");
            throw;
        }
        buffers = (struct buf*)calloc(reqbuf.count, sizeof(*buffers));
        assert(buffers != NULL);

        num_buffers_ = reqbuf.count;
        printf("MMAP Buffer number: %d", num_buffers_);

        struct v4l2_buffer buffer;
        for (unsigned int i = 0; i < reqbuf.count; i++) {
            memset(&buffer, 0, sizeof(buffer));
            buffer.type = reqbuf.type;
            buffer.memory = V4L2_MEMORY_MMAP;
            buffer.index = i;

            this->xioctl(fd_, VIDIOC_QUERYBUF, &buffer);

            buffers[i].length = buffer.length;
            buffers[i].start = mmap(NULL, buffer.length, PROT_READ | PROT_WRITE,
                                    MAP_SHARED, fd_, buffer.m.offset);

            if (MAP_FAILED == buffers[i].start) {
                perror("mmap");
                throw;
            }
        }
    }

    void loop() {
        fd_set fds;
        struct timeval tv;
        int r;
        while (true) {
            // Clear the set of file descriptors to monitor, then add the fd
            // for our device
            FD_ZERO(&fds);
            FD_SET(fd_, &fds);

            // Set the timeout
            tv.tv_sec = 2;
            tv.tv_usec = 0;

            r = select(fd_ + 1, &fds, NULL, NULL, &tv);

            if (-1 == r && errno == EINTR) {
                continue;
            } else if (r == -1) {
                throw;
            } else if (r == 0) {
                fprintf(stderr, "select timeout\n");
                throw;
            } else {
                this->read_frame();
            }
        }
    }

    int read_frame() {
        struct v4l2_buffer buffer;
        memset(&buffer, 0, sizeof(buffer));
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;

        // Dequeue a buffer
        if (-1 == ioctl(fd_, VIDIOC_DQBUF, &buffer)) {
            switch (errno) {
                case EAGAIN:
                    // No buffer in the outgoing queue
                    return 0;
                case EIO:
                    // fall through
                default:
                    perror("VIDIOC_DQBUF");
                    throw;
            }
        }

        assert(buffer.index < num_buffers_);
        process_image(buffers[buffer.index].start);

        // Enqueue the buffer again
        this->xioctl(fd_, VIDIOC_QBUF, &buffer);
        return 1;
    }

    void process_image(const void* pBuffer) {
        this->publisher_->publish("Hello");
        fputc('.', stdout);
        fflush(stdout);
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

    void xioctl(int fd, int request, void* arg) {
        int ret_;
        do {
            ret_ = ioctl(fd, request, arg);
            // retry when write/read operation was interrupted
        } while ((ret_ == -1) && (errno == EINTR));

        // error has occurred
        if (ret_ == -1) {
            perror("ERROR");
            throw;
        }
    }

    struct buf {
        void* start;
        size_t length;
    };
    struct buf* buffers = NULL;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    int fd_;
    unsigned int num_buffers_;
    int width_ = 1920;
    int height_ = 1080;
    int fmt_index_ = 4;

    struct v4l2_capability caps = {};
    struct v4l2_cropcap cropcaps = {};
    // init device
    struct v4l2_fmtdesc fmtdesc = {};
    struct v4l2_format fmt = {};
    // init mmap
    struct v4l2_requestbuffers reqbuf = {};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto cam_node = rclcpp::Node::make_shared("pi_camera");

    try {
        cam_node->init_device();
        cam_node->start_capturing();
    } catch (...) {
        printf("catch block");
    }
    printf("shutdown node");
    rclcpp::shutdown();

    return 0;
}
