cmake_minimum_required(VERSION 3.5)
project(arducam_stereo)

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
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(custom_interfaces REQUIRED)

find_package(OpenCV REQUIRED)
find_library(ARDUCAM_MIPI  arducam_mipicamera PATHS /usr/lib)
include_directories(include ${OpenCV_INCLUDE_DIRS})
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(arducam_stereo src/arducam_stereo.cpp)
target_link_libraries(arducam_stereo ${OpenCV_LIBRARIES} ${ARDUCAM_MIPI})
ament_target_dependencies(arducam_stereo rclcpp  custom_interfaces)

target_include_directories(arducam_stereo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(arducam_stereo_cv src/arducam_stereo_cv.cpp)
target_link_libraries(arducam_stereo_cv ${OpenCV_LIBRARIES} ${ARDUCAM_MIPI})
ament_target_dependencies(arducam_stereo_cv rclcpp image_transport cv_bridge sensor_msgs)

target_include_directories(arducam_stereo_cv PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS arducam_stereo arducam_stereo_cv
  DESTINATION lib/${PROJECT_NAME})

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
