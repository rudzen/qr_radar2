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
#include <cmath>

#include "ControlHeaders.h"
#include "Calculator.h"

static const std::string OPENCV_WINDOW = "QR-Code window";
static const int MAX_VECTOR_SIZE = 4;

#define DBG 1
#define FALSE 0
#define TRUE 1

using namespace std;

class QrRadar {

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber sub_image_;
    ros::Subscriber sub_throttle_;
    ros::Subscriber sub_control_;
#pragma clang diagnostic push
#pragma ide diagnostic ignored "TemplateArgumentsIssues"
    boost::unordered_map<std::string, ros::Time> qr_memory_;
#pragma clang diagnostic pop
    zbar::ImageScanner scanner_;
    ros::Publisher pub_qr_;
    double throttle_; // to slow down the aggresiveness of publishing!!

    std_msgs::String msg_qr_;
    std_msgs::String msg_control_;

    ostringstream stream_qr_;
    vector<CvPoint2D32f> pd_;

    int control;

public:
    QrRadar() : it_(nh_) {
        control = QR_CONTROL_ALL;
        pd_.reserve(MAX_VECTOR_SIZE);
        throttle_ = 2.0; // test value!!!
        scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
        scanner_.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
        // Subscribe to input video feed and publish output video feed
        sub_image_ = it_.subscribe("/ardrone/image_raw", 1, &QrRadar::imageCb, this);
        sub_throttle_ = nh_.subscribe("qr/throttle", 1, &QrRadar::set_throttle, this);
        sub_control_ = nh_.subscribe("qr/control", 1, &QrRadar::set_control, this);
        pub_qr_ = nh_.advertise<std_msgs::String>("qr", 1);
        cv::namedWindow(OPENCV_WINDOW);
    }

    ~QrRadar() {
        cv::destroyWindow(OPENCV_WINDOW);
        sub_image_.shutdown();
        sub_throttle_.shutdown();
        sub_control_.shutdown();
        pub_qr_.shutdown();
        scanner_.~ImageScanner();
        stream_qr_.str(std::string());
        stream_qr_.clear();
    }

    // Function: Image callback from ROS.
    // Description: It is automaticly set up through the class construction.
    void imageCb(const sensor_msgs::ImageConstPtr &msg) {

        //cout << "Timing everything with cv_bridge share & DBG =" << DBG << std::endl;

        /* get the ros time to append to each publish so they can be syncronized from subscription side */
        uint32_t ros_time = ros::Time::now().nsec;

        if (DBG) {
            //cout << "ros time : " << ros_time << endl;
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
        /*
         */
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
            string qr = symbol->get_data();

            /* determine if throttle is enabled, and deny duplicate publishing of same symbol info */
            if (throttle_ > 0.0) {
                if (qr_memory_.count(qr) > 0) {
                    // verify throttle timer to erase it from memory
                    if (ros::Time::now() > qr_memory_.at(qr)) {
                        cout << "Throttle timeout reached, removing data from memory." << endl;;
                        qr_memory_.erase(qr);
                    } else {
                        // timeout was not reached, just move along the found symbols
                        continue;
                    }
                }
                // save the qr code in memory and define new timer for it's erasure
                qr_memory_.insert(make_pair(qr, ros::Time::now() + ros::Duration(throttle_)));
            }

            pd_.clear();
            for (unsigned int i = 0; i < MAX_VECTOR_SIZE; i++) {
                CvPoint2D32f data;
                data.x = symbol->get_location_x(i);
                data.y = symbol->get_location_y(i);
                pd_.push_back(data);
            }

            if (DBG) {
                cout << "Data collected :" << std::endl;
                cout << pd_[0].x << '~' << pd_[0].y << endl;
                cout << pd_[1].x << '~' << pd_[1].y << endl;
                cout << pd_[2].x << '~' << pd_[2].y << endl;
                cout << pd_[3].x << '~' << pd_[3].y << endl;
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
            double qr_width = pd_[2].x - pd_[0].x;
            double qr_height = pd_[1].y - pd_[0].y;
            double z_cm_width = Calculator::distance_z_wall(&qr_width);
            double z_cm_height = Calculator::distance_z_wall(&qr_height);

            cout << "c2c          (pix) : " << distance << endl;
            cout << "distance (w) (cm)  : " << z_cm_width << endl;
            cout << "distance (h) (cm)  : " << z_cm_height << endl;
            cout << "cm offset    (cm)  : " << cm_real << endl;
            cout << "off.hori     (cm)  : " << offset_horizonal << endl;
            cout << "off.vert     (cm)  : " << offset_vertical << endl;

            stream_qr_.str(string());
            stream_qr_.clear();
            stream_qr_ << setfill('0') << setw(10) << ros_time;
            stream_qr_ << '~';
            stream_qr_ << qr;
            stream_qr_ << '~';
            stream_qr_ << setfill('0') << setw(10) << cm_real;
            stream_qr_ << '~';
            stream_qr_ << setfill('0') << setw(10) << offset_horizonal;
            stream_qr_ << '~';
            stream_qr_ << setfill('0') << setw(10) << offset_vertical;
            stream_qr_ << '~';
            stream_qr_ << setfill('0') << setw(10) << z_cm_width;

            /* publish the qr code information */
            msg_qr_.data = stream_qr_.str();
            /*
            if (DBG) {
                cout << "qr_radar sending QR : " << stream_qr_.str() << endl;
            }
            */
            pub_qr_.publish(msg_qr_);
            uint32_t const time_end = ros::Time::now().sec;
            cout << "Time for scanning QR code and calculating (ms) = " << ((time_end - ros_time) / 1000000) << endl;


            if (DBG) {

                // draw lines on image through the center in both x and y
                cv::line(cv_ptr->image, cvPoint(0, cv_ptr->image.rows >> 1), cvPoint(cv_ptr->image.cols, cv_ptr->image.rows >> 1), CV_RGB(255, 255, 255), 1, 8, 0);
                cv::line(cv_ptr->image, cvPoint(cv_ptr->image.cols >> 1, 0), cvPoint(cv_ptr->image.cols >> 1, cv_ptr->image.rows), CV_RGB(255, 255, 255), 1, 8, 0);

                // draw a box around the DETECTED Qr-Code (!!)
                cv::rectangle(cv_ptr->image, cvRect((int) roundf(pd_[0].x), (int) roundf(pd_[0].y), (int) roundf(pd_[2].x - pd_[0].x), (int) roundf(pd_[1].y - pd_[0].y)), CV_RGB(0, 0, 0), 1, 8, 0);

                // write the pixels (width) for the Qr-Code in the lower right side of the image!
                ostringstream tmpss;
                //tmpss << (pd_[2].x - pd_[0].x);
                //cv::Mat m = cv_ptr->image.clone();

                // show the image
                cv::imshow(OPENCV_WINDOW, cv_ptr->image);

                // save the image!!!
                tmpss.str(string());
                tmpss.clear();
                tmpss << "./images/qr_image_";
                tmpss << ros_time;
                tmpss << ".jpg";

                try {
                    cv::imwrite(tmpss.str(), cv_ptr->image);
                    cout << "Saved image file as " << tmpss.str() << endl;
                }
                catch (const std::runtime_error& ex) {
                    cout << "Exception saving image : " << ex.what() << endl;
                }

                cv::waitKey(3);
            }

        }
    }

    void set_throttle(const std_msgs::String::ConstPtr msg) {
        cout << "Got throttle information....";
        try {
            double temp = stod (msg->data.c_str());
            if (temp > 0.0) {
                cout << " changed " << throttle_ << " -> " << temp << endl;
                throttle_ = temp;
            }
        }  catch (const invalid_argument& ia) {
            cout << " but it was invalid : " << ia.what() << endl;
        }  catch (const std::out_of_range& uor) {
            cout << " but it was out of range : " << uor.what() << endl;
        }
    }

    void set_control(const std_msgs::String::ConstPtr msg) {
        cout << "Got control parameter : " << msg->data.c_str() << endl;
        const int control_msg = stoi(msg->data.c_str());
        switch (control_msg) {
            case QR_CONTROL_NONE:
            case QR_CONTROL_ALL:
            case QR_CONTROL_LARGEST:
            case QR_CONTROL_SMALLEST:
            case QR_CONTROL_QUAD_1:
            case QR_CONTROL_QUAD_2:
            case QR_CONTROL_QUAD_3:
            case QR_CONTROL_QUAD_4:
            case QR_CONTROL_RIGHT:
            case QR_CONTROL_LEFT:
            case QR_CONTROL_UPPER:
            case QR_CONTROL_LOWER:
                control = control_msg;
                msg_control_.data = QR_CONTROL_MSG_OK;
            default:
                msg_control_.data = QR_CONTROL_MSG_FAIL;
                break;
        }
        pub_qr_.publish(msg_control_);
    }
};

#endif //QR_RADAR2_QRRADAR_H
