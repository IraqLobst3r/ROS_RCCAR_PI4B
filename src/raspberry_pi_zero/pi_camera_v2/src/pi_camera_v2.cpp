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

#include <chrono>
#include <cstdio>
#include <functional>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/timer.hpp>
#include <std_msgs/msg/detail/header__struct.hpp>
#include <string>

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "image_transport/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

static const char DEVICE[] = "/dev/video0";

int fd;
int seq;
unsigned int num_buffers;
rclcpp::Clock::SharedPtr time_now;
struct buf {
    void* start;
    size_t length;
};
struct buf* buffers = NULL;
struct v4l2_requestbuffers reqbuf = {};

int xioctl(int fd, long unsigned int request, void* arg) {
    int ret_;
    do {
        ret_ = ioctl(fd, request, arg);
        // retry when write/read operation was interrupted
    } while ((ret_ == -1) && (errno == EINTR));

    // error has occurred
    if (ret_ == -1) {
        perror("ERROR");
        return -1;
    }
    return 0;
}

void print_used_format(struct v4l2_format* fmt) {
    char format_code[5];
    strncpy(format_code, (char*)&fmt->fmt.pix.pixelformat, 5);
    printf(
        "Set format:\n"
        " Width: %d\n"
        " Height: %d\n"
        " Pixel format: %s\n"
        " Field: %d\n\n",
        fmt->fmt.pix.width, fmt->fmt.pix.height, format_code,
        fmt->fmt.pix.field);
}

int print_caps() {
    struct v4l2_capability caps = {};
    if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &caps)) {
        return -1;
    }
    printf(
        "Driver Caps:\n"
        "  Driver: \"%s\"\n"
        "  Card: \"%s\"\n"
        "  Bus: \"%s\"\n"
        "  Version: %u.%u.%u\n"
        "  Capabilities: %08x\n",
        caps.driver, caps.card, caps.bus_info, (caps.version >> 16) & 0xFF,
        (caps.version >> 8) & 0xFF, (caps.version) & 0XFF, caps.capabilities);
    return 0;
}

void print_available_formats() {
    struct v4l2_fmtdesc fmtdesc = {};
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    // These will correspond to the flags V4L2_FMT_FLAG_COMPRESSED and
    // V4L2_FMT_FLAG_EMULATED
    char c, e;

    // note: here ioctl() to hide error message
    while (0 == ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc)) {
        c = fmtdesc.flags & 1 ? 'C' : ' ';
        e = fmtdesc.flags & 2 ? 'E' : ' ';

        printf("%2d %c%c %s\n", fmtdesc.index, c, e, fmtdesc.description);
        fmtdesc.index++;
    }
}

int init_device(int fmt_index, int width, int height) {
    struct v4l2_fmtdesc fmtdesc = {};
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    // get format
    fmtdesc.index = fmt_index;
    if (-1 == xioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc)) {
        return -1;
    }

    struct v4l2_format fmt = {};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = fmtdesc.pixelformat;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt)) {
        return -1;
    }
    print_used_format(&fmt);
    return 0;
}

int init_mmap() {
    reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    reqbuf.memory = V4L2_MEMORY_MMAP;
    reqbuf.count = 5;
    if (-1 == xioctl(fd, VIDIOC_REQBUFS, &reqbuf)) {
        return -1;
    }
    if (reqbuf.count < 2) {
        printf("Not enough buffer memory\n");
        return -1;
    }

    buffers = (struct buf*)calloc(reqbuf.count, sizeof(*buffers));
    assert(buffers != NULL);

    num_buffers = reqbuf.count;
    printf("MMAP Buffer number: %d\n", num_buffers);

    struct v4l2_buffer buffer;
    for (unsigned int i = 0; i < reqbuf.count; i++) {
        memset(&buffer, 0, sizeof(buffer));
        buffer.type = reqbuf.type;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index = i;

        if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buffer)) {
            return -1;
        }

        buffers[i].length = buffer.length;
        buffers[i].start = mmap(NULL, buffer.length, PROT_READ | PROT_WRITE,
                                MAP_SHARED, fd, buffer.m.offset);

        if (MAP_FAILED == buffers[i].start) {
            perror("mmap");
            return -1;
        }
    }
    return 0;
}

int start_capturing() {
    enum v4l2_buf_type type;

    struct v4l2_buffer buffer;
    for (unsigned int i = 0; i < num_buffers; i++) {
        // Note that we set bytesused = 0, which will set it to the buffer
        // length
        memset(&buffer, 0, sizeof(buffer));
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index = i;

        // Enqueue the buffer with VIDIOC_QBUF
        if (-1 == xioctl(fd, VIDIOC_QBUF, &buffer)) {
            return -1;
        }
    }

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(fd, VIDIOC_STREAMON, &type)) {
        return -1;
    }
    return 0;
}

void process_image(const void* pBuffer, image_transport::Publisher* pub,
                   int width, int height) {
    cv::Mat image = cv::Mat(height, width, CV_8UC3, (void*)pBuffer);
    seq = seq + 1;
    sensor_msgs::msg::Image::Ptr image_out =
        cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", image).toImageMsg();
    image_out->header.frame_id = "pi_cam";
    image_out->header.stamp = time_now->now();

    pub->publish(image_out);
    fputc('.', stdout);
    fflush(stdout);
}

int read_frame(image_transport::Publisher* pub, int width, int height) {
    struct v4l2_buffer buffer;
    memset(&buffer, 0, sizeof(buffer));
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;

    // Dequeue a buffer
    if (-1 == ioctl(fd, VIDIOC_DQBUF, &buffer)) {
        switch (errno) {
            case EAGAIN:
                // No buffer in the outgoing queue
                return 0;
            case EIO:
                // fall through
            default:
                perror("VIDIOC_DQBUF");
                return -1;
        }
    }

    assert(buffer.index < num_buffers);
    process_image(buffers[buffer.index].start, pub, width, height);

    // Enqueue the buffer again
    if (-1 == xioctl(fd, VIDIOC_QBUF, &buffer)) {
        return -1;
    }
    return 1;
}

int stop_capturing() {
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type)) {
        printf("cannot set stream off");
        return -1;
    }
    return 0;
}

void cleanup() {
    for (unsigned int i = 0; i < reqbuf.count; i++)
        munmap(buffers[i].start, buffers[i].length);
    free(buffers);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto cam_node = rclcpp::Node::make_shared("pi_camera");
    time_now = cam_node->get_clock();

    image_transport::ImageTransport it(cam_node);
    auto publisher = it.advertise("image", 1);

    int width = 1920;
    int height = 1080;
    int fmt_index = 4;

    // open camera file
    fd = open(DEVICE, O_RDWR);
    if (fd < 0) {
        perror(DEVICE);
        return -1;
    }

    cam_node->get_parameter_or("width", width, 1920);
    cam_node->get_parameter_or("height", height, 1080);
    cam_node->get_parameter_or("fmt_index", fmt_index, 4);

    if (0 != print_caps()) {
        printf("ERROR: print_caps");
        close(fd);
        return -1;
    }
    print_available_formats();

    if (0 != init_device(fmt_index, width, height)) {
        printf("ERROR: init device");
        close(fd);
        return -1;
    }

    if (0 != init_mmap()) {
        printf("ERROR: init mmap");
        return -1;
    }

    if (-1 == start_capturing()) {
        printf("ERROR: starting stream");
        cleanup();
        close(fd);
        return -1;
    }

    fd_set fds;
    struct timeval tv;
    int r;
    while (rclcpp::ok()) {
        // Clear the set of file descriptors to monitor, then add the fd
        // for our device
        FD_ZERO(&fds);
        FD_SET(fd, &fds);

        // Set the timeout
        tv.tv_sec = 2;
        tv.tv_usec = 0;

        r = select(fd + 1, &fds, NULL, NULL, &tv);

        if (-1 == r && errno == EINTR) {
            continue;
        } else if (r == -1) {
            return -1;
        } else if (r == 0) {
            fprintf(stderr, "select timeout\n");
            return -1;
        } else {
            if (-1 == read_frame(&publisher, width, height)) {
                return -1;
            }
        }
    }

    if (-1 == stop_capturing()) {
        close(fd);
        return -1;
    }
    cleanup();
    close(fd);

    printf("Shutdown");
    rclcpp::shutdown();

    return 0;
}
