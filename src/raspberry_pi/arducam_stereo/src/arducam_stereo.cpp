#include <iostream>
#include <linux/v4l2-controls.h>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xphoto/white_balance.hpp>
#include <rclcpp/node.hpp>
#include <stdexcept>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <vector>

#include "arducam_mipicamera.h"

#include "custom_interfaces/msg/h264_image.hpp"
#include "rclcpp/rclcpp.hpp"

#define VCOS_ALIGN_DOWN(p, n) (((ptrdiff_t)(p)) & ~((n)-1))
#define VCOS_ALIGN_UP(p, n) VCOS_ALIGN_DOWN((ptrdiff_t)(p) + (n)-1, (n))

int nFrames_ = 0;
int64 t0 = 0;
int64 processingTime = 0;
rclcpp::Node::SharedPtr arducam{nullptr};
rclcpp::Publisher<custom_interfaces::msg::H264Image>::SharedPtr publisher_{nullptr};
std::string frame_id_ = "arducam";

int video_callback(BUFFER* buffer) {
    if (TIME_UNKNOWN == buffer->pts) {
        // Frame data in the second half
    }
    // LOG("buffer length = %d, pts = %llu, flags = 0x%X", buffer->length, buffer->pts,
    // buffer->flags);

    if (buffer->length) {
        if (buffer->flags & MMAL_BUFFER_HEADER_FLAG_CONFIG) {
            // SPS PPS
        }
        if (buffer->flags & MMAL_BUFFER_HEADER_FLAG_CODECSIDEINFO) {
            /// Encoder outputs inline Motion Vectors
        } else {
            // MMAL_BUFFER_HEADER_FLAG_KEYFRAME
            // MMAL_BUFFER_HEADER_FLAG_FRAME_END
            custom_interfaces::msg::H264Image h264_msg;
            h264_msg.header.frame_id = frame_id_;
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
            int64 tp0 = cv::getTickCount();
            /* h264_msg.header.stamp = arducam->now(); */
            h264_msg.data.insert(h264_msg.data.begin(), buffer->data,
                                 (buffer->data + buffer->length));
            publisher_->publish(h264_msg);

            /* if (publisher_->get_subscription_count() > 0) { */
            /*     h264_msg.data.insert(h264_msg.data.begin(), buffer->data, */
            /*                          (buffer->data + buffer->length)); */
            /*     h264_msg.header.stamp = arducam->now(); */
            /*     publisher_->publish(h264_msg); */
            /* } */
            processingTime += cv::getTickCount() - tp0;
            // Here may be just a part of the data, we need to check it.
            if (buffer->flags & MMAL_BUFFER_HEADER_FLAG_FRAME_END) {
            }
        }
    }
    return 0;
}

static void default_status(VIDEO_ENCODER_STATE* state) {
    // Default everything to zero
    memset(state, 0, sizeof(VIDEO_ENCODER_STATE));
    state->encoding = VIDEO_ENCODING_H264;
    state->bitrate = 17000000;
    state->immutableInput = 1; // Flag to specify whether encoder works in place or creates a
                               // new buffer. Result is preview can display either the camera
                               // output or the encoder output (with compression artifacts)
    /**********************H264 only**************************************/
    state->intraperiod = -1; // Not set
                             // Specify the intra refresh period (key frame rate/GoP size).
                             // Zero to produce an initial I-frame and then just P-frames.
    state->quantisationParameter =
        0; // Quantisation parameter. Use approximately 10-40. Default 0 (off)
    state->profile = VIDEO_PROFILE_H264_HIGH; // Specify H264 profile to use for encoding
    state->level = VIDEO_LEVEL_H264_4;        // Specify H264 level to use for encoding
    state->bInlineHeaders = 0;                // Insert inline headers (SPS, PPS) to stream
    state->inlineMotionVectors = 0;           // output motion vector estimates
    state->intra_refresh_type = -1;           // Set intra refresh type
    state->addSPSTiming = 0;                  // zero or one
    state->slices = 1;
    /**********************H264 only**************************************/
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr arducam = rclcpp::Node::make_shared("arducam");
    publisher_ = arducam->create_publisher<custom_interfaces::msg::H264Image>(
        "arducam_stereo/h264_image", 10);

    int _width;
    int _height;
    int res;

    bool _auto_exposure;
    int _exposure_value;
    bool _auto_white_balance;
    bool _auto_wb_compensation;
    int _red_gain;
    int _blue_gain;
    int _scale = 2;

    arducam->declare_parameter<int>("width", 1920);
    arducam->declare_parameter<int>("height", 1080);
    arducam->declare_parameter<int>("scale", 2);
    arducam->declare_parameter<bool>("auto_exposure", true);
    arducam->declare_parameter<int>("exposure_value", 5);
    arducam->declare_parameter<bool>("auto_white_balance", true);
    arducam->declare_parameter<bool>("auto_wb_compensation", true);
    arducam->declare_parameter<int>("red_gain", 10);
    arducam->declare_parameter<int>("blue_gain", 50);

    arducam->get_parameter("width", _width);
    arducam->get_parameter("height", _height);
    arducam->get_parameter("scale", _scale);
    arducam->get_parameter("auto_exposure", _auto_exposure);
    arducam->get_parameter("exposure_value", _exposure_value);
    arducam->get_parameter("auto_white_balance", _auto_white_balance);
    arducam->get_parameter("auto_wb_compensation", _auto_wb_compensation);
    arducam->get_parameter("red_gain", _red_gain);
    arducam->get_parameter("blue_gain", _blue_gain);

    CAMERA_INSTANCE camera_instance;
    VIDEO_ENCODER_STATE video_state;

    RCLCPP_INFO(arducam->get_logger(), "Opening camera...");
    res = arducam_init_camera(&camera_instance);
    if (res) {
        RCLCPP_INFO(arducam->get_logger(), "init camera status = %d", res);
        throw std::runtime_error("Could not init camera");
    }
    RCLCPP_INFO(arducam->get_logger(), "Setting resolution...");
    res = arducam_set_resolution(camera_instance, &_width, &_height);
    if (res) {
        RCLCPP_INFO(arducam->get_logger(), "set resolution status = %d", res);
        throw std::runtime_error("Could not set resolution");
    } else {
        RCLCPP_INFO(arducam->get_logger(), "current resolution is %dx%d", _width, _height);
    }
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
    if (_auto_white_balance) {
        RCLCPP_INFO(arducam->get_logger(), "Enable Auto White Balance...");
        if (arducam_software_auto_white_balance(camera_instance, 1)) {
            RCLCPP_INFO(arducam->get_logger(), "Automatic white balance not supported");
        }
    }
    if (_auto_wb_compensation) {
        RCLCPP_INFO(arducam->get_logger(), "Enable Auto White Balance Compensation...");
        arducam_manual_set_awb_compensation(_red_gain, _blue_gain);
    }

    default_status(&video_state);
    RCLCPP_INFO(arducam->get_logger(), "Start video encoding...");
    res = arducam_set_video_callback(camera_instance, &video_state, video_callback, NULL);
    if (res) {
        RCLCPP_INFO(
            arducam->get_logger(),
            "Failed to start video encoding, probably due to resolution greater than 1920x1080 "
            "or video_state setting error.");
        throw std::runtime_error("Could not start video stream");
    }

    while (rclcpp::ok()) {
        usleep(1000 * 1000 * 10);
    }

    RCLCPP_INFO(arducam->get_logger(), "Stop video encoding...");
    arducam_set_video_callback(camera_instance, NULL, NULL, NULL);

    RCLCPP_INFO(arducam->get_logger(), "Close camera...");
    res = arducam_close_camera(camera_instance);
    if (res)
        RCLCPP_INFO(arducam->get_logger(), "close camera status = %d", res);

    rclcpp::shutdown();
    return 0;
}
