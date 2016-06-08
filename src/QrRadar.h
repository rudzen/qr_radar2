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

#include "PointData.h"
#include "Calculator.h"

static const std::string OPENCV_WINDOW = "Image window";
static const int MAX_VECTOR_SIZE = 4;

static const int DBG = 1;


class QrRadar {

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    boost::unordered_map<std::string, ros::Time> qr_memory_;
    zbar::ImageScanner scanner_;
    ros::Publisher pub_qr_;
    ros::Publisher pub_dist_;
    int count_;
    double throttle_; // to slow down the aggresiveness of publishing!!

    std_msgs::String qr_string;
    std::ostringstream stream_qr;
    std::ostringstream stream_calc;
    std::vector<CvPoint2D32f> pd_;


public:
    QrRadar() : it_(nh_) {
        pd_.reserve(MAX_VECTOR_SIZE);
        throttle_ = 2.0; // test value!!!
        count_ = 0;
        scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
        scanner_.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
        // Subscribe to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/ardrone/image_raw", 1, &QrRadar::imageCb, this);
        pub_qr_ = nh_.advertise<std_msgs::String>("qr", 1);
        pub_dist_ = nh_.advertise<std_msgs::String>("qr_dist", 1);
        cv::namedWindow(OPENCV_WINDOW);
    }

    ~QrRadar() {
        cv::destroyWindow(OPENCV_WINDOW);
        image_sub_.shutdown();
        pub_qr_.shutdown();
        pub_dist_.shutdown();
        scanner_.~ImageScanner();
    }

    // Function: Image callback from ROS.
    // Description: It is automaticly set up through the class construction.
    void imageCb(const sensor_msgs::ImageConstPtr &msg) {

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

        if (scans == 0) {
            if (DBG) {
                //std::cout << "nothing found..." << std::endl;
            }
            return;
        }

        if (DBG) {
            std::cout << "found qr code..." << std::endl;
        }


        /* get the ros time to append to each publish so they can be syncronized from subscription side */
        uint32_t ros_time = ros::Time::now().sec;
        if (DBG) {
            std::cout << "ros time : " << ros_time << std::endl;
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

            // ***************** calculation begins points *********************
            for (int i = 0; i < 4; i++) {
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

            const float half = (pd_[1].x - pd_[0].x) / 2;
            CvPoint2D32f qr_center, im_center;
            CvPoint2D32f line1, line2;

            qr_center.x = pd_[1].x - half;
            qr_center.y = pd_[2].x - half;

            im_center.x = cv_ptr->image.rows >> 1;
            im_center.y = cv_ptr->image.cols >> 1;

            line1.x = pd_[0].x;
            line1.y = im_center.y;

            line2.x = im_center.x;
            line2.y = pd_[0].y;

            double dist_center = Calculator::distance(qr_center, im_center);
            double dist_x = Calculator::distance(line1, line2);

            stream_calc.str(std::string());
            stream_calc.clear();
            stream_calc << std::setfill('0') << std::setw(10) << ros_time;
            stream_calc << '~';
            stream_calc << std::setfill('0') << std::setw(10) << dist_center;
            stream_calc << '~';
            stream_calc << std::setfill('0') << std::setw(10) << dist_x;





            // **************** Qr stuff here *****************************

            stream_qr.str(std::string());
            stream_qr.clear();
            stream_qr << std::setfill('0') << std::setw(10) << ros_time;
            stream_qr << '~';
            stream_qr << qr;

            /* point order is left/top, left/bottom, right/bottom, right/top */
            for (CvPoint2D32f &point : pd_) {
                stream_qr << '~';
                stream_qr << std::setfill('0') << std::setw(5) << point.x;
                stream_qr << '~';
                stream_qr << std::setfill('0') << std::setw(5) << point.y;
            }

            /*
            for (int i = 0; i < 4; i++) {
                stream_qr << std::setfill('0') << std::setw(5) << symbol->get_location_x(i);
                stream_qr << ".";
                stream_qr << std::setfill('0') << std::setw(5) << symbol->get_location_y(i);
            }
             */

            /* publish the qr code information */
            qr_string.data = stream_qr.str();
            if (DBG) {
                std::cout << "qr_radar sending QR : " << stream_qr.str() << std::endl;
            }
            pub_qr_.publish(qr_string);

            if (DBG) {
                std::cout << "qr_radar sending CALC : " << stream_calc.str() << std::endl;
            }
            qr_string.data = stream_calc.str();
            pub_dist_.publish(qr_string);

        }

        if (DBG) {
            cv::imshow(OPENCV_WINDOW, cv_ptr->image);
            cv::waitKey(3);
        }
    }


};

#endif //QR_RADAR2_QRRADAR_H
