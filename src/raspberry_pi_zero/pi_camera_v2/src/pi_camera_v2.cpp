#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/node.hpp>
#include <std_msgs/msg/detail/header__struct.hpp>

#include "cv_bridge/cv_bridge.h"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"
#include "image_transport/publisher.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr cam_node = rclcpp::Node::make_shared("pi_camera");
    image_transport::ImageTransport it(cam_node);
    auto publisher = it.advertise("cam_image", 1);

    /*****************************************************************************************/

    cv::Mat frame;
    printf("Opening camera...\n");
    cv::VideoCapture capture(cv::CAP_V4L);  // open the first camera
    if (!capture.isOpened()) {
        printf("ERROR: Can't initialize camera capture");
        return 1;
    }

    capture.set(cv::CAP_PROP_FRAME_WIDTH, 1920.0);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, 1080.0);
    capture.set(cv::CAP_PROP_MODE, cv::CAP_MODE_BGR);
    printf("Frame width: %f \n", capture.get(cv::CAP_PROP_FRAME_WIDTH));
    printf("     height: %f \n", capture.get(cv::CAP_PROP_FRAME_HEIGHT));
    printf("Capturing FPS: %f \n", capture.get(cv::CAP_PROP_FPS));
    printf("Capturing MODE: %f \n", capture.get(cv::CAP_PROP_MODE));

    size_t nFrames = 0;
    /* bool enableProcessing = false; */
    int64 t0 = cv::getTickCount();
    int64 processingTime = 0;

    int count = 0;
    while (count < 100) {
        capture >> frame;  // read the next frame from camera
        if (frame.empty()) {
            printf("ERROR: Can't grab camera frame.");
            break;
        }
        nFrames++;
        if (nFrames % 10 == 0) {
            const int N = 10;
            int64 t1 = cv::getTickCount();
            std::cout << "Frames captured: "
                      << cv::format("%5lld", (long long int)nFrames)
                      << "    Average FPS: "
                      << cv::format("%9.1f", (double)cv::getTickFrequency() *
                                                 N / (t1 - t0))
                      << "    Average time per frame: "
                      << cv::format("%9.2f ms",
                                    (double)(t1 - t0) * 1000.0f /
                                        (N * cv::getTickFrequency()))
                      << "    Average processing time: "
                      << cv::format("%9.2f ms",
                                    (double)(processingTime)*1000.0f /
                                        (N * cv::getTickFrequency()))
                      << std::endl;
            t0 = t1;
            processingTime = 0;
        }
        int64 tp0 = cv::getTickCount();
        sensor_msgs::msg::Image::SharedPtr image_out =
            cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", frame)
                .toImageMsg();
        image_out->header.frame_id = "cam_image";
        image_out->header.stamp = cam_node->now();

        if (publisher.getNumSubscribers() > 0) {
            publisher.publish(image_out);
        }

        processingTime += cv::getTickCount() - tp0;
        count++;
    }
    std::cout << "Number of captured frames: " << nFrames << std::endl;
    /*****************************************************************************************/
    rclcpp::shutdown();

    return 0;
}
