cmake_minimum_required(VERSION 3.8)
project(test_motor_pkg)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

#find_package(image_transport REQUIRED)
#find_package(cv_bridge REQUIRED)
#find_package(OpenCV REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# must find gstreamer manually
# Find GStreamer
#find_package(PkgConfig REQUIRED)
#pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0>=1.14)
#pkg_check_modules(GSTREAMER_BASE REQUIRED gstreamer-base-1.0)
#pkg_check_modules(GSTREAMER_APP REQUIRED gstreamer-app-1.0)

#include_directories(
#  ${GSTREAMER_INCLUDE_DIRS}
#  ${GSTREAMER_BASE_INCLUDE_DIRS}
#  ${GSTREAMER_APP_INCLUDE_DIRS}
#)
#link_directories(
#  ${GSTREAMER_LIBRARY_DIRS}
#)

# Manually find the libgpiod C++ library
find_library(GPIOD_LIB NAMES gpiodcxx REQUIRED)
include_directories(/usr/include)

find_library(PIGPIO_LIB pigpio REQUIRED)

add_executable(motor_node src/motor_node.cpp)
target_link_libraries(motor_node ${GPIOD_LIB} ${PIGPIO_LIB})
ament_target_dependencies(motor_node rclcpp geometry_msgs)

add_executable(sensor_driver_node src/sensor_driver_node.cpp)
target_link_libraries(sensor_driver_node ${GPIOD_LIB})
ament_target_dependencies(sensor_driver_node rclcpp geometry_msgs)

# Install
install(TARGETS motor_node sensor_driver_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

