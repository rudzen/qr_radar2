/*
 * The MIT License
 *
 * Copyright 2016 Rudy Alex Kohn (s133235@student.dtu.dk).
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
//
// Created by rudz on 6/7/16.
//

#ifndef QR_RADAR2_QRRADAR_H
#define QR_RADAR2_QRRADAR_H

#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
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
#include "Rectangle.h"

#define DBG 1
#define FALSE 0
#define TRUE 1

#define smallest(a, b) (a < b ? a : b)
#define largest(a, b) (a < b ? b : a)

using namespace std;

static const std::string OPENCV_WINDOW = "QR-Code window";
static const int MAX_VECTOR_SIZE = 4;

int display_output = 1;
int scan_images = 1;

class QrRadar {

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber sub_image_;
    ros::Subscriber sub_throttle_;
    ros::Subscriber sub_control_;
    ros::Subscriber sub_display_enable_;
    ros::Subscriber sub_display_disable_;
    ros::Subscriber sub_scan_enable_;
    ros::Subscriber sub_scan_disable_;

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
    vector<v2<int>> pd_;

    int control;

public:
    QrRadar() : it_(nh_) {
        // default control option
        control = QR_CONTROL_ALL;

        // reserve vector space
        pd_.reserve(MAX_VECTOR_SIZE);

        // test value!!!
        throttle_ = 2.0;

        // configure zbar scanner to only allow QR codes (speeds up scan)
        scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
        scanner_.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);

        // subscribe to input video feed and control / throttle topics etc
        sub_image_ = it_.subscribe("/ardrone/image_raw", 1, &QrRadar::imageCb, this);
        sub_throttle_ = nh_.subscribe("qr/throttle", 1, &QrRadar::set_throttle, this);
        sub_control_ = nh_.subscribe("qr/control", 1, &QrRadar::set_control, this);
        sub_display_enable_ = nh_.subscribe("qr/display/enable", 1, &QrRadar::display_enable, this);
        sub_display_disable_ = nh_.subscribe("qr/display/disable", 1, &QrRadar::display_disable, this);
        sub_scan_disable_ = nh_.subscribe("qr/scan/disable", 1, &QrRadar::scan_disable, this);
        sub_scan_enable_ = nh_.subscribe("qr/scan/enable", 1, &QrRadar::scan_enable, this);

        // set result advertisement topic
        pub_qr_ = nh_.advertise<std_msgs::String>("qr", 1);

        // set window (debug) for scanning
        cv::namedWindow(OPENCV_WINDOW);

        cout << "QR-Radar ROS-node initialized..." << endl;
    }

    ~QrRadar() {
        // attempt to clean up nicely..
        cv::destroyWindow(OPENCV_WINDOW);
        sub_image_.shutdown();
        sub_throttle_.shutdown();
        sub_control_.shutdown();
        sub_display_enable_.shutdown();
        sub_display_disable_.shutdown();
        sub_scan_enable_.shutdown();
        sub_scan_disable_.shutdown();
        pub_qr_.shutdown();
        scanner_.~ImageScanner();
        stream_qr_.str(std::string());
        stream_qr_.clear();
    }

    // Function: Image callback from ROS.
    // Description: It is automaticly set up through the class construction.
    void imageCb(const sensor_msgs::ImageConstPtr &msg) {

        // make sure the control unit acts quickly!
        if (control == QR_CONTROL_NONE || scan_images == FALSE) {
            return;
        }

        //cout << "Timing everything with cv_bridge share & DBG =" << DBG << std::endl;

        /* get the ros time to append to each publish so they can be syncronized from subscription side */
        uint32_t ros_time = ros::Time::now().nsec;

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

        // configure scanning area
        intrect img_dim = Calculator::get_img_dim(&control, &cv_ptr->image.cols, &cv_ptr->image.rows);

        // simple whitebalance test
        //Calculator::balance_white(cv_ptr->image);
        // meh

        /* configure image based on available data */
        zbar::Image zbar_image((unsigned int) cv_ptr->image.cols, (unsigned int) cv_ptr->image.rows, "Y800", cv_ptr->image.data, (unsigned long) (cv_ptr->image.cols * cv_ptr->image.rows));

        /* scan the image for QR codes */
        const int scans = scanner_.scan(zbar_image);

        if (scans == FALSE) {
            return;
        }

        int symbol_counter = 0; // for control unit

        /* iterate over located symbols to fetch the data */
        for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin(); symbol != zbar_image.symbol_end(); ++symbol) {

            if (symbol->get_location_size() != MAX_VECTOR_SIZE) {
                // this is not good, it means the QR-Code is not read correctly or some memory corruption has occoured!!!!
                continue;
            }

            ++symbol_counter;

            /* grab the text from the symbol */
            string qr = symbol->get_data();

            /* determine if throttle is enabled, and deny duplicate publishing of same symbol info */
            if (throttle_ > 0.0) {
                if (!qr_memory_.empty() && qr_memory_.count(qr) > 0) {
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

            // ****** QR META STUFF *********
            pd_.clear();
            pd_.push_back(v2<int>(symbol->get_location_x(0), symbol->get_location_y(0)));
            pd_.push_back(v2<int>(symbol->get_location_x(1), symbol->get_location_y(1)));
            pd_.push_back(v2<int>(symbol->get_location_x(2), symbol->get_location_y(2)));
            pd_.push_back(v2<int>(symbol->get_location_x(3), symbol->get_location_y(3)));

            // set dimension for qr (using widest dimensions !!)
            intrect qr_rect(smallest(pd_[0].x, pd_[1].x), smallest(pd_[0].y, pd_[2].y), largest(pd_[2].x, pd_[3].x), largest(pd_[1].y, pd_[2].y));
            if (qr_rect > img_dim) {
                cout << "qr code dimensions are not within controller settings.." << endl;
                return;
            }

            v2<int> qr_c; // this is the main center point from where all calculations are taking place!

            /* precise (based on qr locations from zbar) calculation of center */
            int counter = 0;
            for (v2<int> &point : pd_) {
                qr_c.x += point.x;
                qr_c.y += point.y;
                /* point order is left/top, left/bottom, right/bottom, right/top */
                ++counter;
            }
            qr_c.x /= counter;
            qr_c.y /= counter;

            /* calculate center of image (generic for any size) */
            v2<int> img_c(cv_ptr->image.cols >> 1, cv_ptr->image.rows >> 1);

            /* check if the qr is in a valid position */
            //if (controlbreak(qr_c, img_c) == TRUE) {
            //    cout << "Qr-control blocked processing.. symbol " << symbol_counter << '/' << scans << ".. code : " << control << endl;
            //    continue;
            //}


            int distance_c2c = Calculator::pixel_distance(qr_c, img_c);
            double cm_real = Calculator::pix_to_cm(&distance_c2c);

            v2<double> offsets_cm(Calculator::offset_horizontal(&qr_c.x, &img_c.x, &cm_real), Calculator::offset_vertical(&qr_c.y, &img_c.y, &cm_real));

            // calculate the Z distance_c2c...
            double qr_width_top = pd_[3].x - pd_[0].x;
            double qr_width_buttom =  pd_[2].x - pd_[1].x;
            double qr_width = Calculator::avg(&qr_width_top, &qr_width_buttom);
            double qr_height_left = pd_[1].y - pd_[0].y;
            double qr_height_right = pd_[2].y - pd_[3].y;
            double qr_height = Calculator::avg(&qr_height_left, &qr_height_right);
            v2<double> z_cm_(Calculator::distance_z_wall(&qr_width), Calculator::distance_z_wall(&qr_height));

            // use the smallest distance
            const double z_cm_smallest = smallest(abs(z_cm_.x), abs(z_cm_.y));

            // angular calculations
            const double angle_a = Calculator::angle_a(qr_height, qr_width);
            const double dist_qr_projected = Calculator::dist_qr_projected(qr_height, qr_width, z_cm_smallest, qr_height_left >= qr_height_right ? 1 : -1);
            const double dist_cam_wall = Calculator::dist_wall(qr_height, qr_width, z_cm_smallest);

            // info output
            cout << "Image rect            : " << img_dim << endl;
            cout << "QR rect               : " << qr_rect << endl;
            cout << "Symbol # / total      : " << symbol_counter << '/' << scans << endl;
            cout << "c2c          (pix)    : " << distance_c2c << endl;
            cout << "dist c2c (w) (cm)     : " << z_cm_.x << endl;
            cout << "dist c2c (h) (cm)     : " << z_cm_.y << endl;
            cout << "smallest dist (cm)    : " << z_cm_smallest << endl;
            cout << "cm offset c2c(cm)     : " << cm_real << endl;
            cout << "off.hori     (cm)     : " << offsets_cm.x << endl;
            cout << "off.vert     (cm)     : " << offsets_cm.y << endl;
            cout << "angular a   (deg)     : " << angle_a << endl;
            cout << "dist qr projected (cm): " << dist_qr_projected << endl;
            cout << "dist cam to wall (cm) : " << dist_cam_wall << endl;

            stream_qr_.str(string());
            stream_qr_.clear();
            stream_qr_ << setfill('0') << setw(10) << ros_time;
            stream_qr_ << '~';
            stream_qr_ << qr;
            stream_qr_ << '~';
            stream_qr_ << cm_real;
            stream_qr_ << '~';
            stream_qr_ << offsets_cm.x;
            stream_qr_ << '~';
            stream_qr_ << offsets_cm.y;
            stream_qr_ << '~';
            stream_qr_ << z_cm_smallest;
            stream_qr_ << '~';
            stream_qr_ << angle_a;
            stream_qr_ << '~';
            stream_qr_ << dist_qr_projected;
            stream_qr_ << '~';
            stream_qr_ << dist_cam_wall;

            /* publish the qr code information */
            msg_qr_.data = stream_qr_.str();
            pub_qr_.publish(msg_qr_);
            uint32_t const time_end = ros::Time::now().sec;
            cout << "Time for scanning QR code and calculating (ms) = " << ((time_end - ros_time) / 1000000) << endl;

            if (display_output) {

                // draw lines on image through the center in both x and y
                cv::line(cv_ptr->image, cvPoint(0, cv_ptr->image.rows >> 1), cvPoint(cv_ptr->image.cols, cv_ptr->image.rows >> 1), CV_RGB(255, 255, 255), 1, 8, 0);
                cv::line(cv_ptr->image, cvPoint(cv_ptr->image.cols >> 1, 0), cvPoint(cv_ptr->image.cols >> 1, cv_ptr->image.rows), CV_RGB(255, 255, 255), 1, 8, 0);

                // draw a box around the DETECTED Qr-Code (!!)
                cv::rectangle(cv_ptr->image, cvRect(qr_rect.left, qr_rect.top, qr_rect.right - qr_rect.left, qr_rect.bottom - qr_rect.top), CV_RGB(0, 0, 0), 1, 8, 0);
                cv::circle(cv_ptr->image, cvPoint(pd_[0].x, pd_[0].y), 3, CV_RGB(255, 255, 255));
                cv::circle(cv_ptr->image, cvPoint(pd_[1].x, pd_[1].y), 3, CV_RGB(255, 255, 255));
                cv::circle(cv_ptr->image, cvPoint(pd_[2].x, pd_[2].y), 3, CV_RGB(255, 255, 255));
                cv::circle(cv_ptr->image, cvPoint(pd_[3].x, pd_[3].y), 3, CV_RGB(255, 255, 255));

                ostringstream tmpss;

                tmpss << z_cm_smallest << ' ' << dist_qr_projected;

                cv::putText(cv_ptr->image, tmpss.str(), cvPoint(cv_ptr->image.cols >> 2, cv_ptr->image.rows >> 2), 1, 1, CV_RGB(255, 255, 255));


                // show the image
                cv::imshow(OPENCV_WINDOW, cv_ptr->image);

                // save the image!!!
                tmpss.str(string());
                tmpss.clear();
                tmpss << "./images/qr_image_" << symbol_counter << '_';
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

    int controlbreak(v2<int> qr, v2<int> img) {
        // *********** CONTROL CHECK #1 ***************
        // (for positional check of scanned code)
        switch (control) {
            case QR_CONTROL_UPPER:
                if (qr.y < img.y) {
                    return 0;
                }
            case QR_CONTROL_LOWER:
                if (qr.y > img.y) {
                    return 0;
                }
            case QR_CONTROL_LEFT:
                if (qr.x < img.x) {
                    return 0;
                }
            case QR_CONTROL_RIGHT:
                if (qr.x > img.x) {
                    return 0;
                }
            case QR_CONTROL_QUAD_1:
                if (qr.y < img.y && qr.x > img.x) {
                    return 0;
                }
            case QR_CONTROL_QUAD_2:
                if (qr.y < img.y && qr.x < img.x) {
                    return 0;
                }
            case QR_CONTROL_QUAD_3:
                if (qr.y > img.y && qr.x < img.x) {
                    return 0;
                }
            case QR_CONTROL_QUAD_4:
                if (qr.y > img.y && qr.x > img.x) {
                    return 0;
                }
            case QR_CONTROL_NONE:
                return 1;
            default:break;
        }
        return 1;
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

    void display_enable(const std_msgs::Empty empty) {
        cout << "display enabled.." << endl;
        display_output = 1;
    }

    void display_disable(const std_msgs::Empty empty) {
        cout << "display disabled.." << endl;
        display_output = 0;
    }

    void scan_enable(const std_msgs::String::ConstPtr msg) {
        cout << "scan enabled.." << endl;
        scan_images = 1;
        sub_image_ = it_.subscribe(msg->data.c_str(), 1, &QrRadar::imageCb, this);
    }

    void scan_disable(const std_msgs::Empty empty) {
        cout << "scan disabled.." << endl;
        scan_images = 0;
        sub_image_.shutdown();
    }


};

#endif //QR_RADAR2_QRRADAR_H
