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
//#include "ardrone_autonomy/Navdata.h"
#include <zbar.h>
#include "boost/unordered_map.hpp"
#include <vector>

#include "PointData.h"
#include "Calculator.h"

static const std::string OPENCV_WINDOW = "Image window";

static const int DBG = 1;


class QrRadar {


    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;      //ImageTransport object "it_" under the namespace 'image_transport'
    image_transport::Subscriber image_sub_;       //Subscriber object "image_sub_"
    boost::unordered_map<std::string, ros::Time> qr_memory_;
    zbar::ImageScanner scanner_;
    ros::Publisher qr_pub;
    ros::Publisher qr_dist_pub;
    int count;
    double throttle_; // to slow down the aggresiveness of publishing!!
    std_msgs::String qr_string;
    std::ostringstream qr_stream;
    std::ostringstream calc_stream;

    std::vector<PointData> pd;

public:
    QrRadar() : it_(nh_) {
        pd.reserve(4);
        throttle_ = 2.0; // test value!!!
        count = 0;
        scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
        scanner_.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
        // Subscribe to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/ardrone/image_raw", 1, &QrRadar::imageCb, this);
        qr_pub = nh_.advertise<std_msgs::String>("qr", 1);
        qr_dist_pub = nh_.advertise<std_msgs::String>("qr/dist", 1);
        cv::namedWindow(OPENCV_WINDOW);
    }

    ~QrRadar() {
        cv::destroyWindow(OPENCV_WINDOW);
        image_sub_.shutdown();
        qr_pub.shutdown();
        qr_dist_pub.shutdown();
        scanner_.~ImageScanner();
        pd.clear();
    }

    // Function: Image callback from ROS.
    // Description: It is automaticly set up through the class construction.
    void imageCb(const sensor_msgs::ImageConstPtr &msg) {

        // share test
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        /*
        // create a copy of the image recieved.
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        */

        /* configure image based on available data */
        zbar::Image zbar_image((unsigned int) cv_ptr->image.cols, (unsigned int) cv_ptr->image.rows, "Y800", cv_ptr->image.data, (unsigned long) (cv_ptr->image.cols * cv_ptr->image.rows));

        /* scan the image for QR codes */
        int scans = scanner_.scan(zbar_image);

        if (scans == 0) {
            if (DBG) {
                std::cout << "nothing found..." << std::endl;
            }
            return;
        }
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

            // ***************** calculation begins points *********************
            for (int i = 0; i < 4; i++) {
                PointData pointData(symbol->get_location_x((unsigned int) i), symbol->get_location_y((unsigned int) i));
                pd.push_back(pointData);
            }

            const int half_ = pd[0].x >> 1;
            PointData qr_center(pd[1].x - half_, pd[2].x - half_);
            PointData im_center(cv_ptr->image.rows >> 1, cv_ptr->image.cols >> 1);

            PointData line1(pd[0].x, im_center.y);
            PointData line2(im_center.x, pd[0].y);

            double distance = Calculator::cv_distance(qr_center, im_center);
            double perp_line = Calculator::cv_lineEquation(pd[0], line1, im_center);

            calc_stream.str(std::string());
            calc_stream.clear();
            calc_stream << std::setfill('0') << std::setw(10) << distance;
            calc_stream << std::setfill('0') << std::setw(10) << perp_line;

            // **************** Qr stuff here *****************************

            /* check if it is a location size of 4 (should be the corners of the qr code.) */
            //if (symbol->get_location_size() == 4) {
            qr_stream.str(std::string());
            qr_stream.clear();
            qr_stream << qr;
            qr_stream << ".";

            /* point order is left/top, left/bottom, right/bottom, right/top */
            for (auto &point : pd) {
                qr_stream << std::setfill('0') << std::setw(5) << point.x;
                qr_stream << ".";
                qr_stream << std::setfill('0') << std::setw(5) << point.y;
            }

            /*
            for (int i = 0; i < 4; i++) {
                qr_stream << std::setfill('0') << std::setw(5) << symbol->get_location_x(i);
                qr_stream << ".";
                qr_stream << std::setfill('0') << std::setw(5) << symbol->get_location_y(i);
            }
             */

            /* publish the qr code information */
            qr_string.data = qr_stream.str();
            if (DBG) {
                std::cout << "qr_radar sending QR : " << qr_stream.str() << std::endl;
            }
            qr_pub.publish(qr_string);

            if (DBG) {
                std::cout << "qr_radar sending CALC : " << calc_stream.str() << std::endl;
            }
            qr_string.data = calc_stream.str();
            qr_dist_pub.publish(qr_string);

        }

        if (DBG) {
            cv::imshow(OPENCV_WINDOW, cv_ptr->image);
            cv::waitKey(3);
        }
    }


};

#endif //QR_RADAR2_QRRADAR_H
