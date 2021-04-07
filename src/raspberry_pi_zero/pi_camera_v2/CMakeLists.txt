cmake_minimum_required(VERSION 3.5)
project(pi_camera_v2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set( CMAKE_EXPORT_COMPILE_COMMANDS ON )
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBAV REQUIRED IMPORTED_TARGET
    libavdevice
    libavformat
    libavcodec
    libavutil
    libswscale
)

include_directories(include ${OpenCV_INCLUDE_DIRS})
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(pi_camera_v2_cv src/pi_camera_v2_cv.cpp)
target_link_libraries(pi_camera_v2_cv ${OpenCV_LIBRARIES})
ament_target_dependencies(pi_camera_v2_cv rclcpp std_msgs image_transport cv_bridge sensor_msgs custom_interfaces)

target_include_directories(pi_camera_v2_cv PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(pi_camera_v2_v4l2 src/pi_camera_v2_v4l2.cpp)
target_link_libraries(pi_camera_v2_v4l2 ${OpenCV_LIBRARIES})
ament_target_dependencies(pi_camera_v2_v4l2 rclcpp std_msgs image_transport cv_bridge sensor_msgs custom_interfaces)

target_include_directories(pi_camera_v2_v4l2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(pi_camera_v2_ffmpeg src/pi_camera_v2_ffmpeg.cpp)
target_link_libraries(pi_camera_v2_ffmpeg ${OpenCV_LIBRARIES} PkgConfig::LIBAV)
ament_target_dependencies(pi_camera_v2_ffmpeg rclcpp std_msgs image_transport cv_bridge sensor_msgs custom_interfaces)

target_include_directories(pi_camera_v2_ffmpeg PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS pi_camera_v2_cv pi_camera_v2_v4l2 pi_camera_v2_ffmpeg
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
