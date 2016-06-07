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
#include "ardrone_autonomy/Navdata.h"
#include <zbar.h>
#include "boost/unordered_map.hpp"

static const std::string OPENCV_WINDOW = "Image window";

class QrRadar {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;      //ImageTransport object "it_" under the namespace 'image_transport'
    image_transport::Subscriber image_sub_;       //Subscriber object "image_sub_"
    boost::unordered_map<std::string, ros::Time> qr_memory_;
    zbar::ImageScanner scanner_;
    ros::Publisher qr_pub;
    int count;
    double throttle_; // to slow down the aggresiveness of publishing!!
    std_msgs::String qr_string;
    std::stringstream ss;

public:
    QrRadar() : it_(nh_) {
        throttle_ = 2.0; // test value!!!
        count = 0;
        scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
        scanner_.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
        // Subscribe to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/ardrone/image_raw", 1, &QrRadar::imageCb, this);
        qr_pub = nh_.advertise<std_msgs::String>("qr", 1);
        cv::namedWindow(OPENCV_WINDOW);
    }

    ~QrRadar() {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr &msg) {
        cv_bridge::CvImagePtr cv_ptr;

        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        /* configure image based on available data */
        zbar::Image zbar_image(cv_ptr->image.cols, cv_ptr->image.rows, "Y800", cv_ptr->image.data, cv_ptr->image.cols * cv_ptr->image.rows);

        /* scan the image for QR codes */
        int scans = scanner_.scan(zbar_image);

        if (scans > 0) {

            // TODO Split up for 1 found vs many found
            // TODO Seperate throttle erasure check to inlined function

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

                /* check if it is a location size of 4 (should be the corners of the qr code.) */
                //if (symbol->get_location_size() == 4) {
                ss.str(std::string());
                ss.clear();
                ss << qr;
                ss << ".";
                //n = symbol->get_location_size();
                /* point order is left/top, left/bottom, right/bottom, right/top */
                for (int i = 0; i < 4; i++) {
                    ss << std::setfill('0') << std::setw(5) << symbol->get_location_x(i);
                    ss << ".";
                    ss << std::setfill('0') << std::setw(5) << symbol->get_location_y(i);
                }

                /* publish the qr code information */
                qr_string.data = ss.str();
                std::cout << "qr_radar sending : " << ss.str() << std::endl;
                qr_pub.publish(qr_string);
            }
        } else {
            // std::cout << "nothing found..." << std::endl;
        }


        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::waitKey(3);

        // Output modified video stream
        //image_pub_.publish(cv_ptr->toImageMsg());
    }
};


#endif //QR_RADAR2_QRRADAR_H
