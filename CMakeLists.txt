cmake_minimum_required(VERSION 2.8.3)
project(qr_radar2)

add_definitions(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg cv_bridge roslint image_transport)
find_package(OpenCV REQUIRED)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(qr_radar2 src/QrRadar.cpp src/ControlHeaders.h src/Calculator.h)
target_link_libraries(qr_radar2 zbar ${catkin_LIBRARIES} ${OpenCV_LIBS})

roslint_cpp()

