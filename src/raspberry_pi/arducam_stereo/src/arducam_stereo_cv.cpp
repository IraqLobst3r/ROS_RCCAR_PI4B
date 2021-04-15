#include <iostream>
#include <linux/v4l2-controls.h>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xphoto/white_balance.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <stdexcept>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <vector>

#include "arducam_mipicamera.h"

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#define VCOS_ALIGN_DOWN(p, n) (((ptrdiff_t)(p)) & ~((n)-1))
#define VCOS_ALIGN_UP(p, n) VCOS_ALIGN_DOWN((ptrdiff_t)(p) + (n)-1, (n))

cv::Mat* get_image(CAMERA_INSTANCE camera_instance, int width, int height) {
    IMAGE_FORMAT fmt = {IMAGE_ENCODING_I420, 50};
    BUFFER* buffer = arducam_capture(camera_instance, &fmt, 3000);
    if (!buffer)
        return NULL;

    // The actual width and height of the IMAGE_ENCODING_RAW_BAYER format
    // and the IMAGE_ENCODING_I420 format are aligned,
    // width 32 bytes aligned, and height 16 byte aligned.
    width = VCOS_ALIGN_UP(width, 32);
    height = VCOS_ALIGN_UP(height, 16);
    cv::Mat* image = new cv::Mat(cv::Size(width, (int)(height * 1.5)), CV_8UC1, buffer->data);
    cv::cvtColor(*image, *image, cv::COLOR_YUV2BGR_I420);
    arducam_release_buffer(buffer);
    return image;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    image_transport::Publisher _pub_image_left;
    image_transport::Publisher _pub_image_right;

    rclcpp::Node::SharedPtr arducam_node = rclcpp::Node::make_shared("arducam");
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pub_image =
        arducam_node->create_publisher<sensor_msgs::msg::Image>("arducam", 10);
    /* image_transport::ImageTransport it(arducam_node); */
    /* _pub_image_left = it.advertise("image_left", 1); */
    /* _pub_image_right = it.advertise("image_right", 1); */

    int _width;
    int _height;
    int nFrames_ = 0;
    int64 t0 = 0;
    int64 processingTime = 0;
    int res;

    bool _auto_exposure;
    int _exposure_value;
    bool _auto_white_balance;
    bool _auto_wb_compensation;
    int _red_gain;
    int _blue_gain;
    int _scale = 2;
    CAMERA_INSTANCE camera_instance;

    // Available formats: */
    // mode: 7, 1600x600 */
    // mode: 8, 2560x720 */
    // mode: 9, 3840x1080 */
    // mode: 10, 5184x1944 */
    // mode: 11, 6528x1848 */
    // mode: 12, 6528x2464 */
    arducam_node->declare_parameter<int>("width", 6528);
    arducam_node->declare_parameter<int>("height", 2464);
    arducam_node->declare_parameter<int>("scale", 2);
    arducam_node->declare_parameter<bool>("auto_exposure", true);
    arducam_node->declare_parameter<int>("exposure_value", 5);
    arducam_node->declare_parameter<bool>("auto_white_balance", true);
    arducam_node->declare_parameter<bool>("auto_wb_compensation", true);
    arducam_node->declare_parameter<int>("red_gain", 10);
    arducam_node->declare_parameter<int>("blue_gain", 10);

    arducam_node->get_parameter("width", _width);
    arducam_node->get_parameter("height", _height);
    arducam_node->get_parameter("scale", _scale);
    arducam_node->get_parameter("auto_exposure", _auto_exposure);
    arducam_node->get_parameter("exposure_value", _exposure_value);
    arducam_node->get_parameter("auto_white_balance", _auto_white_balance);
    arducam_node->get_parameter("auto_wb_compensation", _auto_wb_compensation);
    arducam_node->get_parameter("red_gain", _red_gain);
    arducam_node->get_parameter("blue_gain", _blue_gain);

    RCLCPP_INFO(arducam_node->get_logger(), "Opening camera...");
    res = arducam_init_camera(&camera_instance);
    if (res) {
        RCLCPP_INFO(arducam_node->get_logger(), "init camera status = %d", res);
        throw std::runtime_error("Could not init camera");
    }
    RCLCPP_INFO(arducam_node->get_logger(), "Setting resolution...");
    res = arducam_set_resolution(camera_instance, &_width, &_height);
    if (res) {
        RCLCPP_INFO(arducam_node->get_logger(), "set resolution status = %d", res);
        throw std::runtime_error("Could not set resolution");
    } else {
        RCLCPP_INFO(arducam_node->get_logger(), "current resolution is %dx%d", _width, _height);
    }
    if (_auto_exposure) {
        RCLCPP_INFO(arducam_node->get_logger(), "Enabling Auto Exposure...");
        arducam_software_auto_exposure(camera_instance, 1);
    } else {
        if (arducam_set_control(camera_instance, V4L2_CID_EXPOSURE,
                                (int)(_exposure_value * 0xFFFF / 200.0))) {
            RCLCPP_INFO(arducam_node->get_logger(),
                        "Failed to set exposure, the camera may not support this control.");
        }
    }
    if (_auto_white_balance) {
        RCLCPP_INFO(arducam_node->get_logger(), "Enable Auto White Balance...");
        if (arducam_software_auto_white_balance(camera_instance, 1)) {
            RCLCPP_INFO(arducam_node->get_logger(), "Automatic white balance not supported");
        }
    }
    if (_auto_wb_compensation) {
        RCLCPP_INFO(arducam_node->get_logger(), "Enable Auto White Balance Compensation...");
        arducam_manual_set_awb_compensation(_red_gain, _blue_gain);
    }

    while (rclcpp::ok()) {
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
                      << cv::format("%9.2f ms",
                                    (double)(processingTime)*1000.0f / (N * cv::getTickFrequency()))
                      << std::endl;
            t0 = t1;
            processingTime = 0;
        }

        cv::Mat* image = get_image(camera_instance, _width, _height);

        if (!image)
            continue;
        int64 tp0 = cv::getTickCount();

        /* cv::Mat image_clone = image->clone(); */
        /* cv::Mat image_in; */
        /* resize(image_clone, image_in, */
        /*        cv::Size(image_clone.cols / _scale, image_clone.rows / _scale)); */

        /* cv::Mat image_left = image_in(cv::Rect(0, 0, image_in.cols / 2, image_in.rows)); */
        /* cv::Mat image_right = */
        /*     image_in(cv::Rect(image_in.cols / 2, 0, image_in.cols / 2, image_in.rows)); */

        nFrames_ = nFrames_ + 1;
        sensor_msgs::msg::Image::SharedPtr image_out =
            cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", *image).toImageMsg();
        /* sensor_msgs::msg::Image::SharedPtr image_right_out = */
        /*     cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", image_right).toImageMsg(); */

        image_out->header.frame_id = "arducam";
        image_out->header.stamp = arducam_node->now();

        /* image_right_out->header.frame_id = "cam_right"; */
        /* image_right_out->header.stamp = arducam_node->now(); */

        if (_pub_image->get_subscription_count() > 0)
            _pub_image->publish(*image_out);
        /* if (_pub_image_right.getNumSubscribers() > 0) */
        /*     _pub_image_right.publish(image_right_out); */

        delete image;

        processingTime += cv::getTickCount() - tp0;
    }

    RCLCPP_INFO(arducam_node->get_logger(), "Close camera...");
    res = arducam_close_camera(camera_instance);
    if (res)
        RCLCPP_INFO(arducam_node->get_logger(), "close camera status = %d", res);

    rclcpp::shutdown();
    return 0;
}
