#include <condition_variable>
#include <iostream>
#include <linux/v4l2-controls.h>
#include <memory>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xphoto/white_balance.hpp>
#include <queue>
#include <stdexcept>
#include <stdio.h>
#include <string.h>
#include <thread>
#include <time.h>
#include <unistd.h>
#include <vector>

#include "arducam_mipicamera.h"

#include "custom_interfaces/msg/h264_image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

#define VCOS_ALIGN_DOWN(p, n) (((ptrdiff_t)(p)) & ~((n)-1))
#define VCOS_ALIGN_UP(p, n) VCOS_ALIGN_DOWN((ptrdiff_t)(p) + (n)-1, (n))

std::queue<cv::Mat*> mat_queue;
std::condition_variable con_v;
std::mutex mutex_;

int64 nFrames_;
int64 t0 = 0;
int64 processingTime = 0;

std::mutex mutex_run;
bool run = true;
int count_ = 0;
int width_aligned;
int height_aligned;

void get_image(CAMERA_INSTANCE camera_instance) {
    while (run) {

        IMAGE_FORMAT fmt = {IMAGE_ENCODING_I420, 50};
        BUFFER* buffer = arducam_capture(camera_instance, &fmt, 3000);
        if (!buffer)
            continue;

        int64 tp0 = cv::getTickCount();
        {
            std::lock_guard<std::mutex> lk(mutex_);
            cv::Mat* image = new cv::Mat(cv::Size(width_aligned, (int)(height_aligned * 1.5)),
                                         CV_8UC1, buffer->data);
            cv::cvtColor(*image, *image, cv::COLOR_YUV2BGR_I420);
            mat_queue.push(image);
            count_++;
        }
        con_v.notify_all();
        processingTime += cv::getTickCount() - tp0;

        nFrames_++;
        if ((nFrames_ >= 10) && nFrames_ % 10 == 0) {
            const int N = 10;
            int64 t1 = cv::getTickCount();
            std::cout << "Frames captured: " << cv::format("%5lld", (long long int)nFrames_)
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
        arducam_release_buffer(buffer);
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // declare nodeHandle and publisher
    rclcpp::Node::SharedPtr arducam = rclcpp::Node::make_shared("arducam");
    rclcpp::Publisher<custom_interfaces::msg::H264Image>::SharedPtr publisher_ =
        arducam->create_publisher<custom_interfaces::msg::H264Image>("arducam_stereo/h264_image",
                                                                     10);

    // dummy publisher
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr dummy =
        arducam->create_publisher<sensor_msgs::msg::Image>("dummy", 10);

    int _width;
    int _height;
    // belichtungszeit auto empfohlen
    bool _auto_exposure;
    // manuelle belichtungszeit => value/1000 (100us)
    int _exposure_value;
    bool _auto_white_balance;
    bool _auto_wb_compensation;
    int _red_gain;
    int _blue_gain;

    arducam->declare_parameter<int>("width", 3840);
    arducam->declare_parameter<int>("height", 1080);
    arducam->declare_parameter<bool>("auto_exposure", true);
    arducam->declare_parameter<int>("exposure_value", 5);
    arducam->declare_parameter<bool>("auto_white_balance", true);
    arducam->declare_parameter<bool>("auto_wb_compensation", true);
    arducam->declare_parameter<int>("red_gain", 100);
    arducam->declare_parameter<int>("blue_gain", 100);

    arducam->get_parameter("width", _width);
    arducam->get_parameter("height", _height);
    arducam->get_parameter("auto_exposure", _auto_exposure);
    arducam->get_parameter("exposure_value", _exposure_value);
    arducam->get_parameter("auto_white_balance", _auto_white_balance);
    arducam->get_parameter("auto_wb_compensation", _auto_wb_compensation);
    arducam->get_parameter("red_gain", _red_gain);
    arducam->get_parameter("blue_gain", _blue_gain);

    CAMERA_INSTANCE camera_instance;
    int res;
    RCLCPP_INFO(arducam->get_logger(), "Opening camera...");
    res = arducam_init_camera(&camera_instance);
    if (res) {
        RCLCPP_INFO(arducam->get_logger(), "init camera status = %d", res);
        throw std::runtime_error("Could not init camera");
    }

    /******************************************************************************************/
    // Print modes and resolutions
    struct format support_fmt;
    int index = 0;
    char fourcc[5];
    fourcc[4] = '\0';
    while (!arducam_get_support_formats(camera_instance, &support_fmt, index++)) {
        strncpy(fourcc, (char*)&support_fmt.pixelformat, 4);
        RCLCPP_INFO(arducam->get_logger(),
                    "mode: %d, width: %d, height: %d, pixelformat: %s, desc: %s", support_fmt.mode,
                    support_fmt.width, support_fmt.height, fourcc, support_fmt.description);
    }
    index = 0;
    struct camera_ctrl support_cam_ctrl;
    while (!arducam_get_support_controls(camera_instance, &support_cam_ctrl, index++)) {
        int value = 0;
        if (arducam_get_control(camera_instance, support_cam_ctrl.id, &value)) {
            RCLCPP_INFO(arducam->get_logger(), "Get ctrl %s fail.", support_cam_ctrl.desc);
        }
        RCLCPP_INFO(arducam->get_logger(),
                    "index: %d, CID: 0x%08X, desc: %s, min: %d, max: %d, default: %d, current: %d",
                    index - 1, support_cam_ctrl.id, support_cam_ctrl.desc,
                    support_cam_ctrl.min_value, support_cam_ctrl.max_value,
                    support_cam_ctrl.default_value, value);
    }
    /******************************************************************************************/

    // set resolution
    RCLCPP_INFO(arducam->get_logger(), "Setting resolution...");
    res = arducam_set_resolution(camera_instance, &_width, &_height);
    if (res) {
        RCLCPP_INFO(arducam->get_logger(), "set resolution status = %d", res);
        throw std::runtime_error("Could not set resolution");
    } else {
        RCLCPP_INFO(arducam->get_logger(), "current resolution is %dx%d", _width, _height);
    }

    // set exposure auto or manual
    if (_auto_exposure) {
        RCLCPP_INFO(arducam->get_logger(), "Enabling Auto Exposure...");
        arducam_software_auto_exposure(camera_instance, 1);
    } else {
        if (arducam_set_control(camera_instance, V4L2_CID_EXPOSURE,
                                (int)(_exposure_value * 0xFFFF / 200.0))) {
            RCLCPP_INFO(arducam->get_logger(),
                        "Failed to set exposure, the camera may not support this control.");
        }
    }

    // set white balance
    if (_auto_white_balance) {
        RCLCPP_INFO(arducam->get_logger(), "Enable Auto White Balance...");
        if (arducam_software_auto_white_balance(camera_instance, 1)) {
            RCLCPP_INFO(arducam->get_logger(), "Automatic white balance not supported");
        }
    }

    // set red and blue gain
    if (_auto_wb_compensation) {
        RCLCPP_INFO(arducam->get_logger(), "Enable Auto White Balance Compensation...");
        arducam_manual_set_awb_compensation(_red_gain, _blue_gain);
    }

    /**
     * @brief mirror the picture horizontal
     *
     * @see https://www.kernel.org/doc/html/v4.9/media/uapi/v4l/control.html
     */
    RCLCPP_INFO(arducam->get_logger(), "Setting the hfilp...");
    if (arducam_set_control(camera_instance, V4L2_CID_HFLIP, 1)) {
        RCLCPP_INFO(arducam->get_logger(),
                    "Failed to set hflip, the camera may not support this control.");
    }

    width_aligned = VCOS_ALIGN_UP(_width, 32);
    height_aligned = VCOS_ALIGN_UP(_height, 16);

    std::thread cam_thread_(get_image, camera_instance);

    while (rclcpp::ok()) {

        std::unique_lock<std::mutex> lk(mutex_);
        con_v.wait(lk, [] { return count_ > 0; });

        cv::Mat* image = mat_queue.front();
        mat_queue.pop();
        count_--;
        lk.unlock();

        sensor_msgs::msg::Image::SharedPtr image_out =
            cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", *image).toImageMsg();

        image_out->header.frame_id = "raw";
        image_out->header.stamp = arducam->now();

        if (dummy->get_subscription_count() > 0) {
            dummy->publish(*image_out.get());
        }
        delete image;
    }

    mutex_run.lock();
    run = false;
    mutex_run.unlock();

    cam_thread_.join();

    // close camera instance
    RCLCPP_INFO(arducam->get_logger(), "Close camera...");
    res = arducam_close_camera(camera_instance);
    if (res)
        RCLCPP_INFO(arducam->get_logger(), "close camera status = %d", res);

    rclcpp::shutdown();
    return 0;
}
