//
// Created by rudz on 6/7/16.
//

#ifndef QR_RADAR2_QRRADAR_H
#define QR_RADAR2_QRRADAR_H

#include "std_msgs/String.h"
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <zbar.h>
#include "boost/unordered_map.hpp"
#include <vector>
#include <string>

#include "Calculator.h"

static const std::string OPENCV_WINDOW = "QR-Code window";
static const int MAX_VECTOR_SIZE = 4;

#define DBG 1
#define FALSE 0

class QrRadar {

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber sub_image_;
    ros::Subscriber sub_throttle_;
#pragma clang diagnostic push
#pragma ide diagnostic ignored "TemplateArgumentsIssues"
    boost::unordered_map<std::string, ros::Time> qr_memory_;
#pragma clang diagnostic pop
    zbar::ImageScanner scanner_;
    ros::Publisher pub_qr_;
    double throttle_; // to slow down the aggresiveness of publishing!!

    std_msgs::String qr_string_;
    std::ostringstream stream_qr_;
    std::vector<CvPoint2D32f> pd_;

public:
    QrRadar() : it_(nh_) {
        pd_.reserve(MAX_VECTOR_SIZE);
        throttle_ = 2.0; // test value!!!
        scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
        scanner_.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
        // Subscribe to input video feed and publish output video feed
        sub_image_ = it_.subscribe("/ardrone/image_raw", 1, &QrRadar::imageCb, this);
        sub_throttle_ = nh_.subscribe("/qr/throttle", 1, &QrRadar::set_throttle, this);
        pub_qr_ = nh_.advertise<std_msgs::String>("qr", 1);
        cv::namedWindow(OPENCV_WINDOW);
    }

    ~QrRadar() {
        cv::destroyWindow(OPENCV_WINDOW);
        sub_image_.shutdown();
        sub_throttle_.shutdown();
        pub_qr_.shutdown();
        scanner_.~ImageScanner();
        stream_qr_.str(std::string());
        stream_qr_.clear();
    }

    // Function: Image callback from ROS.
    // Description: It is automaticly set up through the class construction.
    void imageCb(const sensor_msgs::ImageConstPtr &msg) {

        std::cout << "Timing everything with cv_bridge copy & DBG =" << DBG << std::endl;

        /* get the ros time to append to each publish so they can be syncronized from subscription side */
        uint32_t ros_time = ros::Time::now().sec;
        if (DBG) {
            std::cout << "ros time : " << ros_time << std::endl;
        }

        // share test
        /*
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
         */

        // create a copy of the image recieved.
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        /* configure image based on available data */
        zbar::Image zbar_image((unsigned int) cv_ptr->image.cols, (unsigned int) cv_ptr->image.rows, "Y800", cv_ptr->image.data, (unsigned long) (cv_ptr->image.cols * cv_ptr->image.rows));

        /* scan the image for QR codes */
        const int scans = scanner_.scan(zbar_image);

        if (scans == FALSE) {
            return;
        }

        /* iterate over located symbols to fetch the data */
        for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin(); symbol != zbar_image.symbol_end(); ++symbol) {

            /* grab the text from the symbol */
            std::string qr = symbol->get_data();

            /* determine if throttle is enabled, and deny duplicate publishing of same symbol info */
            if (throttle_ > 0.0) {
                if (qr_memory_.count(qr) > 0) {
                    // verify throttle timer to erase it from memory
                    if (ros::Time::now() > qr_memory_.at(qr)) {
                        std::cout << "Throttle timeout reached, removing data from memory." << std::endl;;
                        qr_memory_.erase(qr);
                    } else {
                        // timeout was not reached, just move along the found symbols
                        continue;
                    }
                }
                // save the qr code in memory and define new timer for it's erasure
                qr_memory_.insert(std::make_pair(qr, ros::Time::now() + ros::Duration(throttle_)));
            }

            for (unsigned int i = 0; i < MAX_VECTOR_SIZE; i++) {
                CvPoint2D32f data;
                data.x = symbol->get_location_x(i);
                data.y = symbol->get_location_y(i);
                pd_.push_back(data);
            }

            if (DBG) {
                std::cout << "Data collected :" << std::endl;
                std::cout << pd_[0].x << '~' << pd_[0].y << std::endl;
                std::cout << pd_[1].x << '~' << pd_[1].y << std::endl;
                std::cout << pd_[2].x << '~' << pd_[2].y << std::endl;
                std::cout << pd_[3].x << '~' << pd_[3].y << std::endl;
            }

            CvPoint2D32f qr_cent;
            qr_cent.x = 0;
            qr_cent.y = 0;
            int counter = 0;
            for (CvPoint2D32f &point : pd_) {
                qr_cent.x += point.x;
                qr_cent.y += point.y;
                /* point order is left/top, left/bottom, right/bottom, right/top */
                ++counter;
            }
            qr_cent.x /= counter;
            qr_cent.y /= counter;

            /* calculate center of image (generic for any size) */
            CvPoint2D32f img_cent;

            img_cent.x = cv_ptr->image.cols << 1;
            img_cent.y = cv_ptr->image.rows << 1;
            img_cent.x /= counter;
            img_cent.y /= counter;

            double distance = Calculator::distance(qr_cent, img_cent);
            double cm_real = Calculator::pix_to_cm(&distance);

            double offset_horizonal = Calculator::offset_horizontal(&qr_cent.x, &img_cent.x, &cm_real);
            double offset_vertical = Calculator::offset_vertical(&qr_cent.y, &img_cent.y, &cm_real);

            // calculate the Z distance...
            float qr_width = pd_[2].x - pd_[0].x;
            double z_cm = Calculator::distance_z(&qr_width);

            std::cout << "distance  (pix) : " << distance << std::endl;
            std::cout << "distance  (cm)  : " << z_cm << std::endl;
            std::cout << "cm offset (cm)  : " << cm_real << std::endl;
            std::cout << "off.hori  (cm)  : " << offset_horizonal << std::endl;
            std::cout << "off.vert  (cm)  : " << offset_vertical << std::endl;

            stream_qr_.str(std::string());
            stream_qr_.clear();
            stream_qr_ << std::setfill('0') << std::setw(10) << ros_time;
            stream_qr_ << '~';
            stream_qr_ << qr;
            stream_qr_ << '~';
            stream_qr_ << std::setfill('0') << std::setw(10) << cm_real;
            stream_qr_ << '~';
            stream_qr_ << std::setfill('0') << std::setw(10) << offset_horizonal;
            stream_qr_ << '~';
            stream_qr_ << std::setfill('0') << std::setw(10) << offset_vertical;
            stream_qr_ << '~';
            stream_qr_ << std::setfill('0') << std::setw(10) << z_cm;

            /* publish the qr code information */
            qr_string_.data = stream_qr_.str();
            if (DBG) {
                std::cout << "qr_radar sending QR : " << stream_qr_.str() << std::endl;
            }
            pub_qr_.publish(qr_string_);
        }

        if (DBG) {
            cv::imshow(OPENCV_WINDOW, cv_ptr->image);
            cv::waitKey(1);
        }

        uint32_t const time_end = ros::Time::now().sec;
        std::cout << "Time for scanning QR code and calculating (seconds) = " << (time_end - ros_time) << std::endl;
    }

    void set_throttle(const std_msgs::String::ConstPtr msg) {
        std::cout << "Got throttle information....";
        try {
            double temp = std::stod (msg->data.c_str());
            if (temp > 0.0) {
                std::cout << " changed " << throttle_ << " -> " << temp << std::endl;
                throttle_ = temp;
            }
        }  catch (const std::invalid_argument& ia) {
            std::cout << " but it was invalid : " << ia.what() << std::endl;
        }  catch (const std::out_of_range& uor) {
            std::cout << " but it was out of range : " << uor.what() << std::endl;
        }
    }

};

#endif //QR_RADAR2_QRRADAR_H
