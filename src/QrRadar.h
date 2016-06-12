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

#include "ControlHeaders.h"
#include "Calculator.h"
#include "Rectangle.h"
#include "Data.h"
#include "Map.h"

using namespace std;

static const std::string OPENCV_WINDOW = "QR-Code window";
static const int MAX_VECTOR_SIZE = 4;

/*! \brief Main QR-Scanning class.
 *
 *  The main controller class for handling the inputs and outputs
 *  that are associated with this package. It will automaticly
 *  calculate and publish the information gathered from the
 *  subscriped image through the designated callback function(s).
 */
class QrRadar {

    bool display_output = false;
    /*!< Depending on the state, will display output window of scanned QR-code */
    bool scan_images = true;
    /*!< If set to false, any incomming images from the image topic will be ignored */

    ros::NodeHandle nh_;
    /*!< The nodehandler for the topics */
    image_transport::ImageTransport it_;
    /*!< Makes it possible to recieve messages in the form of an image */
    image_transport::Subscriber sub_image_;
    /*!< The subscription object for the image topic */
    ros::Subscriber sub_throttle_;
    /*!< Subscription object for setting the throttle */
    ros::Subscriber sub_control_;
    /*!< Subscription object for setting the control settings */
    ros::Subscriber sub_display_enable_;
    /*!< Subscription object to enable display window */
    ros::Subscriber sub_display_disable_;
    /*!< Subscription object to disable the display window */
    ros::Subscriber sub_scan_enable_;
    /*!< Subscription object to enable scanning of incomming images */
    ros::Subscriber sub_scan_disable_;
    /*!< Subscription object to disable scanning of incomming images */

    //mapqr qr_mapping;

#pragma clang diagnostic push
#pragma ide diagnostic ignored "TemplateArgumentsIssues"
    boost::unordered_map<std::string, ros::Time> qr_memory_;    /*!< Map to keep track of which qr-codes has been sent during the defined throttle interval */
#pragma clang diagnostic pop
    zbar::ImageScanner scanner_;
    /*!< The scanner object which scans for QR-code(s) in a given image */
    ros::Publisher pub_qr_;
    /*!< Publisher for the result(s) gathered from the QR-code */
    ros::Publisher pub_pp_;
    /*!< Publisher for the result(s) gathered from the QR-code to prettyprint */
    ros::Publisher pub_pp_show_;
    /*!< Publisher for the result(s) gathered from the QR-code to prettyprint */

    double throttle_; // to slow down the aggresiveness of publishing!!

    std_msgs::String msg_qr_;
    /*!< String message object for publishing the result */
    std_msgs::Empty msg_pp_show;
    /*!< String message object for publishing the result */
    std_msgs::String msg_control_;
    /*!< String message for async feedback on the state of the throttle changes */

    ostringstream stream_qr_;
    /*!< Output stringstream for gathering the information which is to be published */
    vector<v2<int>> pd_;
    /*!< vector that contains the location of all 4 QR-code corners from the scan */

    int control;                                /*!< Control integer */

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

        sub_image_ = it_.subscribe("/usb_cam/image_raw", 1, &QrRadar::imageCb, this);
        //sub_image_ = it_.subscribe("/ardrone/image_raw", 1, &QrRadar::imageCb, this);

        sub_throttle_ = nh_.subscribe("qr/throttle", 1, &QrRadar::set_throttle, this);
        sub_control_ = nh_.subscribe("qr/control", 1, &QrRadar::set_control, this);
        sub_display_enable_ = nh_.subscribe("qr/display/enable", 1, &QrRadar::display_enable, this);
        sub_display_disable_ = nh_.subscribe("qr/display/disable", 1, &QrRadar::display_disable, this);
        sub_scan_disable_ = nh_.subscribe("qr/scan/disable", 1, &QrRadar::scan_disable, this);
        sub_scan_enable_ = nh_.subscribe("qr/scan/enable", 1, &QrRadar::scan_enable, this);

        // set result advertisement topic
        pub_qr_ = nh_.advertise<std_msgs::String>("qr", 1);
        pub_pp_ = nh_.advertise<std_msgs::String>("prettyprint", 1);
        pub_pp_show_ = nh_.advertise<std_msgs::Empty>("prettyprint/show", 1);

        // set window (debug) for scanning
        cv::namedWindow(OPENCV_WINDOW);

        cout << "QR-Radar ROS-node initialized..." << endl;

        //graph.generate_map();
        //cout << g << endl;
    }

    /*! \brief Brief description.
         Brief description continued.
 *  Detailed description starts here.
 */
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

    /*! \brief Image callback function
     *
     * Is triggered when there is a new image in the topic queue from ROS. It is the main controller function of the entire package,
     *  and thus ties everything together.
     */
    void imageCb(const sensor_msgs::ImageConstPtr &msg) {

        // make sure the control unit acts quickly!
        if (control == QR_CONTROL_NONE || scan_images == false) {
            return;
        }

        if (pub_qr_.getNumSubscribers() == 0) {
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
        intrect img_dim = Calculator::get_img_dim(&control, cv_ptr->image.cols, cv_ptr->image.rows);

        // simple whitebalance test
        //Calculator::balance_white(cv_ptr->image);
        // meh

        /* configure image based on available data */
        zbar::Image zbar_image((unsigned int) cv_ptr->image.cols, (unsigned int) cv_ptr->image.rows, "Y800", cv_ptr->image.data, (unsigned long) (cv_ptr->image.cols * cv_ptr->image.rows));

        /* scan the image for QR codes */
        const int scans = scanner_.scan(zbar_image);

        if (scans == 0) {
            return;
        }

        int symbol_counter = 0; // for control unit

        /* iterate over located symbols to fetch the data */
        for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin(); symbol != zbar_image.symbol_end(); ++symbol) {

            if (symbol->get_location_size() != MAX_VECTOR_SIZE) {
                cout << "FATAL READ ERROR FOR QR-CODE!" << endl;
                // this is not good, it means the QR-Code is not read correctly or some memory corruption has occoured!!!!
                continue;
            }

            ++symbol_counter;

            /* grab the text from the symbol */
            string qr_string = symbol->get_data();

            /* determine if throttle is enabled, and deny duplicate publishing of same symbol info */
            if (throttle_ > 0.0) {
                if (!qr_memory_.empty() && qr_memory_.count(qr_string) > 0) {
                    // verify throttle timer to erase it from memory
                    if (ros::Time::now() > qr_memory_.at(qr_string)) {
                        cout << "Throttle timeout reached, removing data from memory." << endl;;
                        qr_memory_.erase(qr_string);
                    } else {
                        // timeout was not reached, just move along the found symbols
                        continue;
                    }
                }
                // save the qr code in memory and define new timer for it's erasure
                qr_memory_.insert(make_pair(qr_string, ros::Time::now() + ros::Duration(throttle_)));
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

            /* set the image center (generic for any size) */
            v2<int> img_c(cv_ptr->image.cols >> 1, cv_ptr->image.rows >> 1);

            /* check if the qr is in a valid position */
            // currently not used...

            //if (controlbreak(qr_c, img_c) == TRUE) {
            //    cout << "Qr-control blocked processing.. symbol " << symbol_counter << '/' << scans << ".. code : " << control << endl;
            //    continue;
            //}

            int distance_c2c = Calculator::pixel_distance(qr_c, img_c);
            double cm_real = Calculator::pix_to_cm(&distance_c2c);

            // set the offset in cm from qr center to image center in for both x & y
            v2<double> offsets_cm(Calculator::offset_horizontal(&qr_c.x, &img_c.x, &cm_real), Calculator::offset_vertical(&qr_c.y, &img_c.y, &cm_real));

            // populate the data class, this will automaticly calculate the needed bits and bobs
            ddata qr(pd_[3].x - pd_[0].x, pd_[2].x - pd_[1].x, pd_[1].y - pd_[0].y, pd_[2].y - pd_[3].y);

            cout << "Time for scanning QR code and calculating (ms) = " << ((ros::Time::now().sec - ros_time) / 1000000) << '\n';

            // info output
            cout << "Image rect            : " << img_dim << '\n';
            cout << "QR rect               : " << qr_rect << '\n';
            cout << "Symbol # / total      : " << symbol_counter << '/' << scans << '\n';
            cout << "c2c          (pix)    : " << distance_c2c << '\n';
            cout << "smallest dist (cm)    : " << qr.dist_z << '\n';
            cout << "cm offset c2c(cm)     : " << cm_real << '\n';
            cout << "off.hori     (cm)     : " << offsets_cm.x << '\n';
            cout << "off.vert     (cm)     : " << offsets_cm.y << '\n';
            cout << "angular a   (deg)     : " << qr.angle << '\n';
            cout << "dist qr projected (cm): " << qr.dist_z_projected << '\n';
            cout << "dist cam to wall (cm) : " << qr.dist_z_cam_wall << '\n';

            //qr_mapping.set_visited(&qr_string, &qr.dist_z);

            if (pub_qr_.getNumSubscribers() > 0) {
                stream_qr_.str(string());
                stream_qr_.clear();
                stream_qr_ << setfill('0') << setw(10) << ros_time;
                stream_qr_ << '~';
                stream_qr_ << qr_string;
                stream_qr_ << '~';
                stream_qr_ << offsets_cm.x;
                stream_qr_ << '~';
                stream_qr_ << offsets_cm.y;
                stream_qr_ << '~';
                stream_qr_ << qr;


                /* publish the qr code information */
                msg_qr_.data = stream_qr_.str();
                pub_qr_.publish(msg_qr_);
                pub_pp_.publish(msg_qr_);
                pub_pp_show_.publish(msg_pp_show);
            }


            if (display_output) {

                // draw lines on image through the center in both x and y
                cv::line(cv_ptr->image, cvPoint(0, cv_ptr->image.rows >> 1), cvPoint(cv_ptr->image.cols, cv_ptr->image.rows >> 1), CV_RGB(255, 255, 255));
                cv::line(cv_ptr->image, cvPoint(cv_ptr->image.cols >> 1, 0), cvPoint(cv_ptr->image.cols >> 1, cv_ptr->image.rows), CV_RGB(255, 255, 255));

                // draw a box around the DETECTED Qr-Code (!!)
                cv::line(cv_ptr->image, cvPoint(pd_[0].x, pd_[0].y), cvPoint(pd_[1].x, pd_[1].y), CV_RGB(0, 0, 0));
                cv::line(cv_ptr->image, cvPoint(pd_[0].x, pd_[0].y), cvPoint(pd_[3].x, pd_[3].y), CV_RGB(0, 0, 0));
                cv::line(cv_ptr->image, cvPoint(pd_[2].x, pd_[2].y), cvPoint(pd_[1].x, pd_[1].y), CV_RGB(0, 0, 0));
                cv::line(cv_ptr->image, cvPoint(pd_[2].x, pd_[2].y), cvPoint(pd_[3].x, pd_[3].y), CV_RGB(0, 0, 0));

                //cv::rectangle(cv_ptr->image, cvRect(qr_rect.left, qr_rect.top, qr_rect.right - qr_rect.left, qr_rect.bottom - qr_rect.top), CV_RGB(0, 0, 0), 1, 8, 0);
                cv::circle(cv_ptr->image, cvPoint(pd_[0].x, pd_[0].y), 3, CV_RGB(255, 255, 255));
                cv::circle(cv_ptr->image, cvPoint(pd_[1].x, pd_[1].y), 3, CV_RGB(255, 255, 255));
                cv::circle(cv_ptr->image, cvPoint(pd_[2].x, pd_[2].y), 3, CV_RGB(255, 255, 255));
                cv::circle(cv_ptr->image, cvPoint(pd_[3].x, pd_[3].y), 3, CV_RGB(255, 255, 255));

                ostringstream tmpss;

                tmpss << qr.dist_z << ' ' << qr.dist_z_projected;

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
                    cout << "Saved image file as " << tmpss.str() << '\n';
                }
                catch (const std::runtime_error& ex) {
                    cout << "Exception saving image : " << ex.what() << '\n';
                }

                cv::waitKey(3);
            }
        }
    }

    /*! \brief Checks if control setting is in effect
    *
    * When the control is defined (through the related topis), this function will check if the corresponding control setting
    * violates the current qr code in relation to the image.
    */
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
            default:
                break;
        }
        return 1;
    }

    /*! \brief Set throttle callback function through topic
    *
    * Will try to set the incomming throttle value, it guards against unwanted information and will
    * not allow invalid values.
    */
    void set_throttle(const std_msgs::String::ConstPtr msg) {
        cout << "Got throttle information....";
        try {
            double temp = stod(msg->data.c_str());
            if (temp > 0.0) {
                cout << " changed " << throttle_ << " -> " << temp << endl;
                throttle_ = temp;
            }
        } catch (const invalid_argument &ia) {
            cout << " but it was invalid : " << ia.what() << endl;
        } catch (const std::out_of_range &uor) {
            cout << " but it was out of range : " << uor.what() << endl;
        }
    }

    /*! \brief Set control callback function through topic
    *
    * Will try to set the incomming control value, it guards against unwanted information and will
    * not allow invalid values. It will always respond with a message indicating success or fauilure.
    */
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

    /*! \brief Enables display output
    *
    * Enables the display_output setting for this node.
    */
    void display_enable(const std_msgs::Empty empty) {
        cout << "display enabled.." << endl;
        display_output = true;
    }

    /*! \brief Disables display output
    *
    * Disables and closes any open output window open for this node.
    */
    void display_disable(const std_msgs::Empty empty) {
        cout << "display disabled.." << endl;
        display_output = false;
        cv::destroyAllWindows();
    }

    /*! \brief Enables QR scanning
    *
    * Disables current image subscription and enables parsed topic.
    */
    void scan_enable(const std_msgs::String::ConstPtr msg) {
        cout << "scan enabled.." << endl;
        scan_images = true;
        sub_image_.shutdown();
        sub_image_ = it_.subscribe(msg->data.c_str(), 1, &QrRadar::imageCb, this);
    }

    /*! \brief Disables QR scanning
    *
    * Disables current image subscription.
    */
    void scan_disable(const std_msgs::Empty empty) {
        cout << "scan disabled.." << endl;
        scan_images = false;
        sub_image_.shutdown();
    }


};

#endif //QR_RADAR2_QRRADAR_H
