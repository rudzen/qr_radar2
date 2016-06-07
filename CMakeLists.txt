cmake_minimum_required(VERSION 2.8.3)
project(qr_radar2)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg cv_bridge roslint image_transport)
find_package(OpenCV REQUIRED)

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(qr_radar2 src/QrRadar.cpp)
target_link_libraries(qr_radar2 zbar ${catkin_LIBRARIES} ${OpenCV_LIBS})
#add_dependencies(qr_radar2 qr_radar2_generate_messages_cpp)

roslint_cpp()

