#
# The MIT License
#
# Copyright 2016 Rudy Alex Kohn (s133235@student.dtu.dk).
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
cmake_minimum_required(VERSION 2.8.3)
project(qr_radar2)

add_definitions(-std=c++11)
add_definitions(-DUSE_RESTRICT)
add_definitions(-O3)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg cv_bridge roslint image_transport)
find_package(OpenCV REQUIRED)

find_package(Boost)
if (Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIR})
    add_definitions( "-DHAS_BOOST" )
endif()

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(qr_radar2 src/QrRadar.cpp src/Data.h src/Vec.h src/Rectangle.h src/ControlHeaders.h src/Calculator.h src/PrettyPrint.h)
target_link_libraries(qr_radar2 zbar ${catkin_LIBRARIES} ${OpenCV_LIBS})

roslint_cpp()

