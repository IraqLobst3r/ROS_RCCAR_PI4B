#include <iostream>
#include <linux/v4l2-controls.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xphoto/white_balance.hpp>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <vector>

#include "raspicam_stereo/arducam_mipicamera.h"

#include "custom_interfaces/msg/h264_image.hpp"
#include "rclcpp/rclcpp.hpp"

#define VCOS_ALIGN_DOWN(p, n) (((ptrdiff_t)(p)) & ~((n)-1))
#define VCOS_ALIGN_UP(p, n) VCOS_ALIGN_DOWN((ptrdiff_t)(p) + (n)-1, (n))

int main(int argc, char** argv) {
    (void)argc;
    (void)argv;

    printf("hello world arducam_stereo package\n");
    return 0;
}
