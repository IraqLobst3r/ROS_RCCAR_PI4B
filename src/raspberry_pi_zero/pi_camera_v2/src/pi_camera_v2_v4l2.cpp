#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <opencv2/core/types.hpp>
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

#include "custom_interfaces/msg/h264_image.hpp"
#include "cv_bridge/cv_bridge.h"
/* #include "image_transport/image_transport.hpp" */
/* #include "image_transport/publisher.hpp" */
#include "rclcpp/rclcpp.hpp"
/* #include "std_msgs/msg/string.hpp" */

using namespace std::chrono_literals;

static const char DEVICE[] = "/dev/video0";

class PiCameraV4l2 : public rclcpp::Node {
  public:
    PiCameraV4l2(int fd) : Node("pi_camera_v4l2"), fd_(fd) {
        publisher_ =
            this->create_publisher<custom_interfaces::msg::H264Image>("pi_cam/h264_image", 10);

        this->declare_parameter<int>("width", 1920);
        this->declare_parameter<int>("height", 1080);
        this->declare_parameter<int>("fmt_index", 4);
        this->declare_parameter<bool>("fmt_grey", false);
        this->declare_parameter<int>("mmap_req_buffer_num", 5);

        this->get_parameter("width", width_);
        this->get_parameter("height", height_);
        this->get_parameter("fmt_index", fmt_index_);
        this->get_parameter("fmt_grey", fmt_grey_);
        this->get_parameter("mmap_req_buffer_num", req_buffer_num_);
    }
    int init_device() {
        struct v4l2_fmtdesc fmtdesc = {};
        fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmtdesc.index = fmt_index_;

        // get format
        if (-1 == xioctl(VIDIOC_ENUM_FMT, &fmtdesc)) {
            perror("VIDIOC_ENUM_FMT");
            return -1;
        }

        struct v4l2_format fmt = {};
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        fmt.fmt.pix.width = width_;
        fmt.fmt.pix.height = height_;
        if (fmt_grey_) {
            fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
        } else {
            fmt.fmt.pix.pixelformat = fmtdesc.pixelformat;
        }
        fmt.fmt.pix.field = V4L2_FIELD_NONE;

        if (-1 == xioctl(VIDIOC_S_FMT, &fmt)) {
            perror("VIDIOC_S_FMT");
            return -1;
        }
        print_used_format(&fmt);
        if (-1 == this->init_mmap()) {
            return -1;
        }
        return 0;
    }

    int start_capturing() {
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
            if (-1 == xioctl(VIDIOC_QBUF, &buffer)) {
                perror("VIDIOC_QBUF");
                return -1;
            }
        }

        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (-1 == xioctl(VIDIOC_STREAMON, &type)) {
            perror("VIDIOC_STREAMON");
            return -1;
        }
        t0 = cv::getTickCount();
        return 0;
    }

    /* void set_publisher(image_transport::Publisher* pub) { publisher_ = pub; }
     */

    int stream_cam() {
        // Clear the set of file descriptors to monitor, then add the fd
        // for our device
        FD_ZERO(&fds);
        FD_SET(fd_, &fds);

        // Set the timeout
        tv.tv_sec = 2;
        tv.tv_usec = 0;

        r = select(fd_ + 1, &fds, NULL, NULL, &tv);

        if (-1 == r && errno == EINTR) {
            return 0;
        } else if (r == -1) {
            printf("ERROR: select \n");
            return -1;
        } else if (r == 0) {
            fprintf(stderr, "select timeout\n");
            return -1;
        } else {
            if (-1 == read_frame()) {
                return -1;
            }
        }

        return 0;
    }

    int stop_capturing() {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (-1 == xioctl(VIDIOC_STREAMOFF, &type)) {
            perror("VIDIOC_STREAMOFF");
            return -1;
        }
        return 0;
    }

    void cleanup() {
        printf("INFO: free buffer");
        for (unsigned int i = 0; i < reqbuf.count; i++)
            munmap(buffers[i].start, buffers[i].length);
        free(buffers);
    }

    int print_caps() {
        struct v4l2_capability caps = {};
        if (-1 == xioctl(VIDIOC_QUERYCAP, &caps)) {
            perror("VIDIOC_QUERYCAP");
            return -1;
        }
        printf("Driver Caps:\n"
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
        while (0 == ioctl(fd_, VIDIOC_ENUM_FMT, &fmtdesc)) {
            c = fmtdesc.flags & 1 ? 'C' : ' ';
            e = fmtdesc.flags & 2 ? 'E' : ' ';

            printf("%2d %c%c %s\n", fmtdesc.index, c, e, fmtdesc.description);
            fmtdesc.index++;
        }
    }

  private:
    int xioctl(long unsigned int request, void* arg) {
        int ret_;
        do {
            ret_ = ioctl(fd_, request, arg);
            // retry when write/read operation was interrupted
        } while ((ret_ == -1) && (errno == EINTR));

        // error has occurred
        if (ret_ == -1) {
            return -1;
        }
        return 0;
    }

    int init_mmap() {
        reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        reqbuf.memory = V4L2_MEMORY_MMAP;
        reqbuf.count = req_buffer_num_;
        if (-1 == xioctl(VIDIOC_REQBUFS, &reqbuf)) {
            perror("VIDIOC_REQBUFS");
            return -1;
        }
        if (reqbuf.count < 2) {
            printf("Not enough buffer memory\n");
            return -1;
        }

        buffers = (struct buf*)calloc(reqbuf.count, sizeof(*buffers));
        assert(buffers != NULL);

        num_buffers_ = reqbuf.count;
        printf("MMAP Buffer number: %d\n", num_buffers_);

        struct v4l2_buffer buffer;
        for (unsigned int i = 0; i < reqbuf.count; i++) {
            memset(&buffer, 0, sizeof(buffer));
            buffer.type = reqbuf.type;
            buffer.memory = V4L2_MEMORY_MMAP;
            buffer.index = i;

            if (-1 == xioctl(VIDIOC_QUERYBUF, &buffer)) {
                perror("VIDIOC_QUERYBUF");
                return -1;
            }

            buffers[i].length = buffer.length;
            buffers[i].start =
                mmap(NULL, buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buffer.m.offset);

            if (MAP_FAILED == buffers[i].start) {
                perror("MMAP");
                return -1;
            }
        }
        return 0;
    }

    int read_frame() {
        struct v4l2_buffer buffer;
        memset(&buffer, 0, sizeof(buffer));
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;

        // Dequeue a buffer
        if (-1 == xioctl(VIDIOC_DQBUF, &buffer)) {
            switch (errno) {
            case EAGAIN:
                // No buffer in the outgoing queue
                return -2;
            case EIO:
                // fall through
            default:
                perror("VIDIOC_DQBUF");
                return -1;
            }
        }

        assert(buffer.index < num_buffers_);

        nFrames++;
        if (nFrames % 10 == 0) {
            const int N = 10;
            int64 t1 = cv::getTickCount();
            std::cout << "Frames captured: " << cv::format("%5lld", (long long int)nFrames)
                      << "    Average FPS: "
                      << cv::format("%9.1f", (double)cv::getTickFrequency() * N / (t1 - t0))
                      << "    Average time per frame: "
                      << cv::format("%9.2f ms",
                                    (double)(t1 - t0) * 1000.0f / (N * cv::getTickFrequency()))
                      << "    Average processing time: "
                      << cv::format("%9.2f ms",
                                    (double)(processingTime)*1000.0f / (N * cv::getTickFrequency()))
                      << std::endl;
            t0 = t1;
            processingTime = 0;
        }

        int64 tp0 = cv::getTickCount();
        process_image(buffers[buffer.index].start, buffers[buffer.index].length);
        processingTime += cv::getTickCount() - tp0;

        // Enqueue the buffer again
        if (-1 == xioctl(VIDIOC_QBUF, &buffer)) {
            perror("VIDIOC_QBUF");
            return -1;
        }
        return 0;
    }

    void process_image(const void* pBuffer, const size_t len) {
        custom_interfaces::msg::H264Image h264_msg;
        h264_msg.header.frame_id = "pi_cam";
        h264_msg.header.stamp = this->now();
        h264_msg.seq = nFrames;
        size_t length = len / sizeof(uint8_t);

        std::vector<uint8_t> image((const uint8_t*)pBuffer, (const uint8_t*)pBuffer + length);
        h264_msg.data.insert(h264_msg.data.begin(), &image[0], &image[image.size()]);
        /* cv::Mat image = cv::Mat(cv::Size(width_, height_), CV_8UC3,
         * (void*)pBuffer); */

        /* sensor_msgs::msg::Image::SharedPtr image_out = */
        /*     cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8",
         * image).toImageMsg(); */

        /* image_out->header.frame_id = "pi_cam"; */
        /* image_out->header.stamp = this->now(); */

        if (publisher_->get_subscription_count() > 0) {
            publisher_->publish(h264_msg);
        }
    }

    void print_used_format(struct v4l2_format* fmt) {
        char format_code[5];
        strncpy(format_code, (char*)&fmt->fmt.pix.pixelformat, 5);
        printf("Set format:\n"
               " Width: %d\n"
               " Height: %d\n"
               " Pixel format: %s\n"
               " Field: %d\n\n",
               fmt->fmt.pix.width, fmt->fmt.pix.height, format_code, fmt->fmt.pix.field);
    }

    struct buf {
        void* start;
        size_t length;
    };
    struct buf* buffers = NULL;
    struct v4l2_requestbuffers reqbuf = {};

    fd_set fds;
    struct timeval tv;
    int r;

    int fd_;
    int width_;
    int height_;
    int fmt_index_;
    unsigned int num_buffers_;
    int req_buffer_num_;
    size_t nFrames = 0;
    bool fmt_grey_;
    int64 t0 = 0;
    int64 processingTime = 0;
    /* image_transport::Publisher* publisher_; */
    rclcpp::Publisher<custom_interfaces::msg::H264Image>::SharedPtr publisher_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // open camera file
    int fd;
    fd = open(DEVICE, O_RDWR);
    if (fd < 0) {
        perror(DEVICE);
        return -1;
    }
    std::shared_ptr<PiCameraV4l2> cam_node = std::make_shared<PiCameraV4l2>(fd);

    /* image_transport::ImageTransport it(cam_node); */
    /* image_transport::Publisher publisher = it.advertise("image", 1); */

    if (0 != cam_node->print_caps()) {
        printf("ERROR: print_caps");
        close(fd);
        return -1;
    }

    cam_node->print_available_formats();

    if (0 != cam_node->init_device()) {
        printf("ERROR: init device");
        close(fd);
        return -1;
    }

    if (-1 == cam_node->start_capturing()) {
        printf("ERROR: starting stream");
        cam_node->cleanup();
        close(fd);
        return -1;
    }

    /* cam_node->set_publisher(&publisher); */

    int count = 0;
    while (count < 100 && rclcpp::ok()) {
        if (-1 == cam_node->stream_cam()) {
            break;
        }
        count++;
    }

    if (-1 == cam_node->stop_capturing()) {
        cam_node->cleanup();
        close(fd);
        return -1;
    }

    cam_node->cleanup();
    close(fd);

    printf("Shutdown");
    rclcpp::shutdown();

    return 0;
}
