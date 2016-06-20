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
#include "std_msgs/Byte.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "boost/unordered_map.hpp"
#include <vector>
#include <string>
#include <zbar.h>

#include "ControlHeaders.h"
#include "Calculator.h"
#include "Rectangle.h"
#include "Data.h"
#include "PrettyPrint.h"
#include "Q.h"

using namespace std;

static const std::string OPENCV_WINDOW = "QR-Code window";
static const int MAX_VECTOR_SIZE = 4;
static const char pubSeperator = ' ';

static const string VERSION = "0.3.0";


/*! \brief Main QR-Scanning class.
 *
 *  The main controller class for handling the inputs and outputs
 *  that are associated with this package. It will automaticly
 *  calculate and publish the information gathered from the
 *  subscriped image through the designated callback function(s).
 */
class QrRadar {

private:

    bool controlling = true;
    bool left = false, right = false;

    const string searcher = "rosservice call /ardrone/setflightanimation 7 450";

    //const string topicFrontCamera = "/usb_cam/image_raw";
    const string topicFrontCamera = "/ardrone/front/image_raw";
    const string topicButtomCamera = "/ardrone/bottom/image_raw";

    std::map<bool, string> cameraWallTopic;

    bool shouldDisplayDebugWindow = false;                           /*!< Depending on the state, will display output window of scanned QR-code */
    bool isScanEnabled = true;                                      /*!< If set to false, any incomming images from the image topic will be ignored */
    float throttle_;                                                /*!<Control the rate to publish identical QR-codes */
    int control;                                                    /*!< Control integer */

    ros::NodeHandle nhQR = ros::NodeHandle("/qr");                  /*!< The nodehandler for the topics */
    ros::NodeHandle nhScan;
    ros::NodeHandle nhScanSetWall;
    ros::NodeHandle nhThrottle;
    ros::NodeHandle nhDisplay;
    ros::NodeHandle nhCollision;

    //ros::NodeHandle nhHover;

    image_transport::ImageTransport imageTransport;                 /*!< Makes it possible to recieve messages in the form of an image */
    image_transport::Subscriber subImage;                           /*!< The subscription object for the image topic */
    ros::Subscriber subThrottle;                                    /*!< Subscription object for setting the throttle */
    //ros::Subscriber subControl;                                     /*!< Subscription object for setting the control settings */
    ros::Subscriber subDisplayEnable;                               /*!< Subscription object to enable display window */
    ros::Subscriber subDisplayDisable;                              /*!< Subscription object to disable the display window */
    ros::Subscriber subDisplaySet;                                  /*!< Subscription object to enable / disable image display of scanned QR-code */
    ros::Subscriber subScanTopic;                                   /*!< Subscription object to enable scanning of incomming images */
    ros::Subscriber subScanDisable;                                 /*!< Subscription object to disable scanning of incomming images */
    ros::Subscriber subScanWall;                                    /*!< Subscription for switching between wall and floor mode.*/
    ros::Subscriber subScanSet;                                     /*!< Subscription object to enable / disable scanning of incomming images */
    ros::Subscriber subScanSetWall;                                 /*!< Subscription object to enable / disable specific walls */
    ros::Subscriber subKaffe;                                       /*!< Subscription object to make coffeeeeeeeeee!!!! */

#pragma clang diagnostic push
#pragma ide diagnostic ignored "TemplateArgumentsIssues"
    boost::unordered_map<std::string, ros::Time> qr_memory;        /*!< Map to keep track of which qr-codes has been sent during the defined throttle interval */
#pragma clang diagnostic pop


    ros::Publisher pubQR;                                           /*!< Publisher for the result(s) gathered from the QR-code */
    ros::Publisher pubCollision;                                    /*!< Publisher for potential collision with wall detection by QR-code distance */
    ros::Publisher pubScanCount;                                    /*!< Publisher for no detection of QR-codes in scanned image */
    //ros::Publisher pubHover;

    std_msgs::String msg_qr_;                                       /*!< String message object for publishing the result */
    std_msgs::String msg_control_;                                  /*!< String message for async feedback on the state of the throttle changes */
    std_msgs::String msg_collision;
    std_msgs::String msg_scan_count;
    geometry_msgs::Twist hover; // test hover

    ostringstream streamQR;                                         /*!< Output stringstream for gathering the information which is to be published */
    vector<v2<int>> qrLoc;                                          /*!< vector that contains the location of all 4 QR-code corners from the scan */
    Calculator c;
    map<char, bool> enabled_qr_codes;

    /* set up zbar objects */

    zbar::ImageScanner imageScanner;
    zbar::Image zImage;

    Queue<string> print_queue;
    Queue<string> qr_queue;

    boost::thread* t_printer;
    boost::thread* t_qrpub;
    //boost::thread* t_hover;

    uint32_t globalcount = 0;
    uint32_t globalfirst = ros::Time::now().nsec;

public:

    QrRadar() : imageTransport(nhQR) {

        if (system("clear") != 0) {
            cout << "Error while calling system command clear." << endl;
        }
        PrettyPrint pp(VERSION);
        pp.show_menu();

        cout << "Configuring opencv.. ";

        if (!cv::useOptimized()) {
            cv::setUseOptimized(true);
        }

        cv::setNumThreads(4); // let opencv use 4 threads for processing stuff.

        // set window (debug) for scanning
        cv::namedWindow(OPENCV_WINDOW);

        cout << "done" << endl;
        cout << "Configuring default scan variables.. ";

        // hover stuff.
        hover.angular.x = 0;
        hover.angular.y = 0;
        hover.angular.z = 0;
        hover.linear.x = 0;
        hover.linear.y = 0;
        hover.linear.z = 0;

        // default control option
        control = QR_CONTROL_ALL;

        // reserve vector space
        qrLoc.reserve(MAX_VECTOR_SIZE);

        // interval between (in ros time) sending identical qr-code information, based on it's text
        throttle_ = 2;

        /* configure default dis-/enabled qr_codes */
        enabled_qr_codes['0'] = true;
        enabled_qr_codes['1'] = false;
        enabled_qr_codes['2'] = true;
        enabled_qr_codes['3'] = false;

        /* configure camera mapping */
        cameraWallTopic[true] = topicFrontCamera;
        cameraWallTopic[false] = topicButtomCamera;

        cout << "done" << endl;
        cout << "Configuring zbar.. ";


        // configure zbar scanner to only allow QR codes (speeds up scan quite a bit!)
        imageScanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
        imageScanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
        imageScanner.enable_cache(false);
        zImage.set_format("Y800"); // or GRAY

        cout << "done" << endl;
        cout << "Configuring ROS objects..";

        /* configure nodehandle sub namespaces */
        nhScan = ros::NodeHandle(nhQR, "scan");
        nhScanSetWall = ros::NodeHandle(nhScan, "set/wall");
        nhThrottle = ros::NodeHandle(nhQR, "throttle");
        nhDisplay = ros::NodeHandle(nhQR, "display");
        nhCollision = ros::NodeHandle(nhQR, "/collision");
        //nhHover = ros::NodeHandle("cmd_vel");

        // subscribe to input video feed and control / throttle topics etc
        subImage = imageTransport.subscribe(topicFrontCamera, 1, &QrRadar::imageCb, this);

        subThrottle = nhThrottle.subscribe("set", 1, &QrRadar::throttle_set, this);
        subDisplayEnable = nhDisplay.subscribe("enable", 1, &QrRadar::display_enable, this);
        subDisplayDisable = nhDisplay.subscribe("disable", 1, &QrRadar::display_disable, this);
        subDisplaySet = nhDisplay.subscribe("set", 1, &QrRadar::display_set, this);
        subScanTopic = nhScan.subscribe("topic", 1, &QrRadar::topic_set, this);
        subScanWall = nhScan.subscribe("mode", 1, &QrRadar::scan_flip, this);
        subScanSet = nhScan.subscribe("set", 1, &QrRadar::scan_set, this);
        subScanSet = nhScan.subscribe("set/wall", 1, &QrRadar::scan_set_wall, this);
        subKaffe = nhQR.subscribe("kaffe", 1, &QrRadar::kaffe, this);

        // Set publishers
        pubQR = nhQR.advertise<std_msgs::String>(nhQR.getNamespace(), 1);
        pubCollision = nhCollision.advertise<std_msgs::String>("wall", 1);
        pubScanCount = nhQR.advertise<std_msgs::String>("count", 1);

        //pubHover = nhHover.advertise<geometry_msgs::Twist>("cmd_vel", 1);

        /* configure automated threads */
        t_printer = new boost::thread(boost::bind(&QrRadar::printer, this));
        t_qrpub = new boost::thread(boost::bind(&QrRadar::qr_publisher, this));
        //t_hover = new boost::thread(boost::bind(&QrRadar::hover_drone, this));

        cout << "done" << endl;
        cout << "Ready.." << endl;
    }

    ~QrRadar() {
        // attempt to clean up nicely..
        delete t_printer;
        delete t_qrpub;
        //delete t_hover;
        cv::destroyWindow(OPENCV_WINDOW);
        pubQR.shutdown();
        pubCollision.shutdown();
        pubScanCount.shutdown();
        //pubHover.shutdown();
        subImage.shutdown();
        subThrottle.shutdown();
        //subControl.shutdown();
        subDisplayEnable.shutdown();
        subDisplayDisable.shutdown();
        subScanTopic.shutdown();
        subScanDisable.shutdown();
        subScanWall.shutdown();
        subScanSet.shutdown();
        subScanSetWall.shutdown();
        subDisplaySet.shutdown();
        subKaffe.shutdown();
        streamQR.str(std::string());
        streamQR.clear();
    }

    /*! \brief Image callback function
     *
     * Is triggered when there is a new image in the topic queue from ROS. It is the main controller function of the entire package,
     *  and thus ties everything together.
     */
    void imageCb(const sensor_msgs::ImageConstPtr &msg) {

        // make sure the control unit acts quickly!
        if (control == QR_CONTROL_NONE || !isScanEnabled) {
            return;
        }

        /* get the ros time to append to each publish so they can be syncronized from subscription side */
        uint32_t ros_time_end;
        uint32_t ros_time = ros::Time::now().nsec;

        // share the image
        //cv_bridge::CvImageConstPtr cv_ptr;

        // create a copy of the image recieved.
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // testing distance with bigger image + equalizeHist + sharpening
        cv::Mat dest = cv::Mat(cv_ptr->image.rows, cv_ptr->image.cols, cv_ptr->image.type());
        //cv::resize(cv_ptr->image, dest, cvSize(cv_ptr->image.cols << 1, cv_ptr->image.rows << 1), 0, 0, CV_INTER_LINEAR);

        cv::GaussianBlur(cv_ptr->image, dest, cv::Size(0, 0), 3);
        cv::addWeighted(cv_ptr->image, 1.5, dest, -0.5, 0, dest);
        cv::equalizeHist(dest, cv_ptr->image);

        //cv::bilateralFilter(dest, cv_ptr->image, 5, 1, 2);
        //cv_ptr->image = dest.clone();
        //dest.release();

        // configure scanning area
        intrect img_dim = Calculator::get_img_dim(&control, cv_ptr->image.cols, cv_ptr->image.rows);

        /* configure image based on available data */
        zImage.set_size((unsigned int) cv_ptr->image.cols, (unsigned int) cv_ptr->image.rows);
        zImage.set_data(cv_ptr->image.data, (unsigned long) (cv_ptr->image.cols * cv_ptr->image.rows));

        /* scan the image for QR codes */
        const int scans = imageScanner.scan(zImage);

        // publish amount of QR codes located.
        if (pubScanCount.getNumSubscribers() > 0) {
            streamQR.str(string());
            streamQR.clear();
            streamQR << ros_time << pubSeperator << scans;
            msg_scan_count.data = streamQR.str();
            pubScanCount.publish(msg_scan_count);
        }

        /*
        if (scans == 0 && controlling) {
            if (!left && !right) {
                ROS_INFO("Controlling mode active!");
                right = true;
            }
            if (hover.angular.z < 20 && right) {
                ROS_INFO("RIGHT angular += 1");
                hover.angular.z += 1;
                if (hover.angular.z >= 20) {
                    ROS_INFO("X Angular reached 20+, switching to LEFT mode");
                    right = false;
                    left = true;
                }
            }
            if (hover.angular.z > -20 && left) {
                ROS_INFO("LEFT angular -= 1");
                hover.angular.z -= 1;
                if (hover.angular.z <= -20) {
                    ROS_INFO("LEFT mode target reached, switching to RIGHT mode");
                    right = true;
                    left = false;
                }
            }
            pubHover.publish(hover);
            return;
          */
        if (scans == 0) {
            return;
        }
        //controlling = false;
        //hover.angular.z = 0;

        int symbol_counter = 0; // for control unit

        /* iterate over located symbols to fetch the data */
        for (zbar::Image::SymbolIterator symbol = zImage.symbol_begin(); symbol != zImage.symbol_end(); ++symbol) {

            if (symbol->get_location_size() != MAX_VECTOR_SIZE) {
                cout << "FATAL READ ERROR FOR QR-CODE!" << endl;
                // this is not good, it means the QR-Code is not read correctly or some memory corruption has occoured!!!!
                continue;
            }

            ++symbol_counter;

            /* grab the text from the symbol */
            string qr_string = symbol->get_data();

            // guard for not reading airfields...
            if (qr_string.at(0) == 'A') {
                continue;
            }

            /* determine if throttle is enabled, and deny duplicate publishing of same symbol info */
            if (throttle_ > 0.0) {
                if (!qr_memory.empty() && qr_memory.count(qr_string) > 0) {
                    // verify throttle timer to erase it from memory
                    if (ros::Time::now() > qr_memory.at(qr_string)) {
                        ROS_INFO("Throttle timeout reached, removing data from memory.");
                        qr_memory.erase(qr_string);
                    } else {
                        // timeout was not reached, just move along the found symbols
                        continue;
                    }
                }
                // save the qr code in memory and define new timer for it's erasure
                qr_memory.insert(make_pair(qr_string, ros::Time::now() + ros::Duration(throttle_)));
            }

            // ****** QR META STUFF *********
            qrLoc.clear();
            /* location order is left/top, left/bottom, right/bottom, right/top */
            qrLoc.push_back(v2<int>(symbol->get_location_x(0), symbol->get_location_y(0)));
            qrLoc.push_back(v2<int>(symbol->get_location_x(1), symbol->get_location_y(1)));
            qrLoc.push_back(v2<int>(symbol->get_location_x(2), symbol->get_location_y(2)));
            qrLoc.push_back(v2<int>(symbol->get_location_x(3), symbol->get_location_y(3)));

            // set dimension for qr (using widest dimensions !!)
            intrect qr_rect(smallest(qrLoc[0].x, qrLoc[1].x), smallest(qrLoc[0].y, qrLoc[2].y), largest(qrLoc[2].x, qrLoc[3].x), largest(qrLoc[1].y, qrLoc[2].y));

            /*
            if (qr_rect > img_dim) {
                cout << "qr code dimensions are not within controller settings.." << endl;
                return;
            }
             */

            v2<int> qr_c; // this is the main center point from where all calculations are taking place!

            /* precise (based on qr locations from zbar) calculation of center */
            for (v2<int> &point : qrLoc) {
                qr_c.x += point.x;
                qr_c.y += point.y;
            }
            qr_c.x /= qrLoc.size();
            qr_c.y /= qrLoc.size();

            /* set the image center (generic for any size) */
            v2<int> img_c(cv_ptr->image.cols >> 1, cv_ptr->image.rows >> 1);

            /* check if the qr is in a valid position */
            // currently not used...
            //if (controlbreak(qr_c, img_c) == TRUE) {
            //    cout << "Qr-control blocked processing.. symbol " << symbol_counter << '/' << scans << ".. code : " << control << endl;
            //    continue;
            //}

            int width_top = qrLoc[3].x - qrLoc[0].x;
            int width_bottom = qrLoc[2].x - qrLoc[1].x;
            int height_left = qrLoc[1].y - qrLoc[0].y;
            int height_right = qrLoc[2].y - qrLoc[3].y;

            // populate the data class, this will automaticly calculate the needed bits and bobs
            ddata qr(width_top, width_bottom, height_left, height_right, c);

            qr.offsets.x = (qr_c.x - img_c.x) * c.pix_to_cm((width_top + width_bottom) >> 1);
            qr.offsets.y = (qr_c.y - img_c.y) * c.pix_to_cm((height_left + height_right) >> 1);

            // if the angle is 0, set the dist_z_projected to x offset (makes it a bit more precise)
            if (qr.angle == 0) {
                qr.dist_z_projected = qr.offsets.x;
            }

            // publish collision warning right away!
            if (pubCollision.getNumSubscribers() > 0 && c.wall_mode && qr.dist_z_cam_wall <= 200 && qr_string.at(0) == 'W') {
                streamQR.str(string());
                streamQR.clear();
                streamQR << qr.dist_z_cam_wall;
                msg_collision.data = streamQR.str();
                pubCollision.publish(msg_collision);
                if (system("rosservice call /ardrone/setledanimation 9 3 1") != 0) {
                    print_queue.push("Error while calling animation service");
                }
            }
            qr.room_coords = c.getCoordinatePosition(&qr_string, &qr.dist_z, &qr.dist_z_projected);

            qr.dist_wall_0 = c.get00Distance(&qr_string, &qr.dist_z, &qr.dist_z_projected);
            qr.dist_wall_1 = c.get01Distance(&qr_string, &qr.dist_z, &qr.dist_z_projected);
            qr.dist_wall_2 = c.get02Distance(&qr_string, &qr.dist_z, &qr.dist_z_projected);
            qr.dist_wall_3 = c.get03Distance(&qr_string, &qr.dist_z, &qr.dist_z_projected);

            streamQR.str(string());
            streamQR.clear();
            // info output
            streamQR << "Text                  : " << qr_string << '\n';
            streamQR << "Image rect            : " << img_dim << '\n';
            streamQR << "QR rect               : " << qr_rect << '\n';
            streamQR << "Symbol # / total      : " << symbol_counter << '/' << scans << '\n';
            //cout << "c2c          (pix)    : " << distance_c2c << '\n';
            streamQR << "smallest dist (cm)    : " << qr.dist_z << '\n';
            //cout << "cm offset c2c(cm)     : " << cm_real << '\n';
            streamQR << "off.hori     (cm)     : " << qr.offsets.x << '\n';
            streamQR << "off.vert     (cm)     : " << qr.offsets.y << '\n';
            streamQR << "angular a   (deg)     : " << qr.angle << '\n';
            streamQR << "dist qr projected (cm): " << qr.dist_z_projected << '\n';
            streamQR << "dist cam to wall (cm) : " << qr.dist_z_cam_wall << '\n';
            streamQR << "0 wall dist           : " << qr.dist_wall_0 << '\n';
            streamQR << "1 wall dist           : " << qr.dist_wall_1 << '\n';
            streamQR << "2 wall dist           : " << qr.dist_wall_2 << '\n';
            streamQR << "3 wall dist           : " << qr.dist_wall_3 << '\n';
            streamQR << "------------------------\n";

            /*
            if (c.wall_mode) {
                streamQR << "Dist. to wall behind : " << c.getBackWallDistance(&qr_string.at(2), &qr.dist_z_cam_wall) << '\n';
                streamQR << "Dist. to DRONE-LEFT wall : " << c.getLeftWallDistance(&qr_string, &qr.dist_z_projected, &qr.angle) << '\n';
                streamQR << "Dist. to DRONE-RIGHT wall : " << c.getRightWallDistance(&qr_string, &qr.dist_z_projected, &qr.angle) << '\n';
                streamQR << "Room coord (x,y)  : " << pubSeperator << qr.room_coords << '\n';
            } else {
                streamQR << "Dist. to ceiling :  " << c.getCeilingDistance(&qr.dist_z_cam_wall) << '\n';
            }
            */
            print_queue.push(streamQR.str());
            //cout << endl;

            /* If anyone is listening .... AND ... PTAM does NOT work well with glass walls, so those are skipped */
            if (pubQR.getNumSubscribers() > 0) {//} && enabled_qr_codes[qr_string.at(2)]) {
                streamQR.str(string());
                streamQR.clear();
                streamQR << ros_time << pubSeperator;
                streamQR << qr_string << pubSeperator;
                streamQR << qr;

                /*
                // additional calculations done ONLY FOR THIS PARTICULAR CONTEST!!
                if (c.wall_mode) {
                    streamQR << pubSeperator << c.getBackWallDistance(&qr_string.at(2), &qr.dist_z_cam_wall);
                    streamQR << pubSeperator << c.getLeftWallDistance(&qr_string, &qr.dist_z_projected, &qr.angle);
                    streamQR << pubSeperator << c.getRightWallDistance(&qr_string, &qr.dist_z_projected, &qr.angle);
                    streamQR << pubSeperator << qr.room_coords.x;
                    streamQR << pubSeperator << qr.room_coords.y;
                } else {
                    streamQR << pubSeperator << c.getCeilingDistance(&qr.dist_z_cam_wall);
                    streamQR << pubSeperator << '0';
                    streamQR << pubSeperator << '0';
                    streamQR << pubSeperator << '0';
                    streamQR << pubSeperator << '0';
                }
                 */

                /* publish the qr code information */
                qr_queue.push(streamQR.str());
            }

            if (shouldDisplayDebugWindow) {
                createQRImage(cv_ptr, &qr, &symbol_counter, &ros_time);
            }

            streamQR.str(string());
            streamQR.clear();
            ros_time_end = ros::Time::now().nsec;
            streamQR << "Global QR-codes found :" << globalcount++ << " the last : " << c.nanoToMili(ros_time_end - globalfirst);
            print_queue.push(streamQR.str());

            cout << "Time for scan/calc/show of complete image (ms) = " << c.nanoToMili(ros_time_end - ros_time) << '\n';
        }

        imageScanner.recycle_image(zImage);
    }
/*
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
    void hover_drone() {
        geometry_msgs::Twist t;
        t.angular.x = t.angular.y = t.angular.z = 0;
        t.linear.x = t.linear.y = t.linear.z = 0;
        while (!t_hover->interruption_requested()) {
            pubHover.publish(t);
            boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
            ROS_INFO("SPAMMING HOVER!!!");
        }
    }
#pragma clang diagnostic pop
*/
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
    void printer() {
        while (!t_printer->interruption_requested()) {
            cout << QrRadar::print_queue.pop() << endl;
        }
    }
#pragma clang diagnostic pop

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
    void qr_publisher() {
        while (!t_qrpub->interruption_requested()) {
            QrRadar::msg_qr_.data = QrRadar::qr_queue.pop();
            pubQR.publish(QrRadar::msg_qr_);
            if (system("rosservice call /ardrone/setledanimation 3 3 1") != 0) {
                print_queue.push("Error while calling animation service.");
            }
            //ROS_INFO("Attempting to hover..");
            //pubHover.publish(hover);
        }
    }
#pragma clang diagnostic pop

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
    void throttle_set(const std_msgs::Float32 msg) {
        cout << "throttle command recieved.. ";
        if (msg.data >= 0.0) {
            if (msg.data == throttle_) {
                cout << "but was un-altered : " << throttle_ << '\n';
            } else {
                throttle_ = msg.data;
                cout << " new value is : " << throttle_ << '\n';
            }
        } else {
            cout << "invalid value : " << throttle_ << '\n';
        }
    }

    /*! \brief Set control callback function through topic
    *
    * Will try to set the incomming control value, it guards against unwanted information and will
    * not allow invalid values. It will always respond with a message indicating success or fauilure.
    */
    /*
    void control_set(const std_msgs::Byte msg) {
        cout << "control command recieved.. ";
        if (msg.data == control) {
            cout << "but was un-altered : " << throttle_ << endl;
            return;
        }
        switch (msg.data) {
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
                control = msg.data;
                msg_control_.data = QR_CONTROL_MSG_OK;
                break;
            default:
                msg_control_.data = QR_CONTROL_MSG_FAIL;
                break;
        }
        pubQR.publish(msg_control_);
    }
     */

    /*! \brief Enables display output
    *
    * Enables the shouldDisplayDebugWindow setting for this node.
    */
    void display_enable(const std_msgs::Empty empty) {
        cout << "display enabled.." << endl;
        shouldDisplayDebugWindow = true;
    }

    /*! \brief Disables display output
    *
    * Disables and closes any open output window open for this node.
    */
    void display_disable(const std_msgs::Empty empty) {
        cout << "display disabled.." << endl;
        shouldDisplayDebugWindow = false;
        cv::destroyAllWindows();
    }

    void display_set(const std_msgs::Bool msg) {
        cout << "display_set command recieved... ";
        if (msg.data != shouldDisplayDebugWindow) {
            shouldDisplayDebugWindow = msg.data;
            cout << "new value : " << shouldDisplayDebugWindow << '\n';
        } else {
            cout << "value already set : " << shouldDisplayDebugWindow << '\n';
        }
    }

    void display_flip(const std_msgs::Empty msg) {
        shouldDisplayDebugWindow ^= true;
        cout << "image display configured to : o" << (shouldDisplayDebugWindow ? "n" : "ff") << '\n';
    }

    void kaffe(const std_msgs::Empty msg) {
        cout << "1 sec dude, skal lige brygge den sorte....\n";
        sleep(1);
        cout << "          / / / /\n";
        cout << "   _______\\\\\\\\_______ \n";
        cout << "  /                     \\\n";
        cout << " \\                      / ___\n";
        cout << "  |                      |/  \\\n";
        cout << "  |    Kaf               |    /\n";
        cout << "  |       Kaf            |   /\n";
        cout << "  |          Kaffe!!     |  /\n";
        cout << "  |                      |_/\n";
        cout << "  |                      |\n";
        cout << " \\_____________________/\n\n" << endl;
    }

    /*! \brief Dis-/enables QR scanning
    *
    * Disables or enables current image subscription.
    */
    void scan_set(const std_msgs::Bool msg) {
        cout << "scan_set command recieved... ";
        if (msg.data != isScanEnabled) {
            isScanEnabled = msg.data;
            cout << "new value : " << isScanEnabled << '\n';
        } else {
            cout << "value already set : " << isScanEnabled << '\n';
        }
    }

    void scan_set_wall(const std_msgs::String msg) {
        if (msg.data.length() == 1) {
            stringstream ss(msg.data);
            string item;
            // disable all
            enabled_qr_codes['0'] = false;
            enabled_qr_codes['1'] = false;
            enabled_qr_codes['2'] = false;
            enabled_qr_codes['3'] = false;
            while (getline(ss, item, ' ')) {
                enabled_qr_codes[item.at(0)] = true;
            }

            // just for debugging
            typedef map<char, bool>::iterator it_type;
            for(it_type iterator = enabled_qr_codes.begin(); iterator != enabled_qr_codes.end(); ++iterator) {
                cout << "wall " << iterator->first << " set to " << iterator->second;
            }
            return;
        }
        cout << "invalid string sent to /qr/scan/wall/set" << '\n';
    }

    /*! \brief Set image topic
    *
    * Disables current image subscription and enables parsed topic.
    */
    void topic_set(const std_msgs::String::ConstPtr msg) {
        isScanEnabled = true;
        subImage.shutdown();
        subImage = imageTransport.subscribe(msg->data.c_str(), 1, &QrRadar::imageCb, this);
        cout << "scan enabled.. topic set to : " << msg->data.c_str() << '\n';
    }

    void scan_flip(const std_msgs::Empty empty) {
        c.wall_mode ^= true;
        // now flip the camera topic !
        switchCamera(c.wall_mode);
        cout << "new scan mode selected : " << (c.wall_mode ? "wall" : "floor") << '\n';
    }

    void switchCamera(bool wallMode) {
        subImage.shutdown();
        subImage = imageTransport.subscribe(cameraWallTopic[wallMode], 1, &QrRadar::imageCb, this);
    }

    void createQRImage( cv_bridge::CvImagePtr cv_ptr,ddata *qr, int *symbol_count, uint32_t *ros_time) {
        // draw lines on image through the center in both x and y
        cv::line(cv_ptr->image, cvPoint(0, cv_ptr->image.rows >> 1), cvPoint(cv_ptr->image.cols, cv_ptr->image.rows >> 1), CV_RGB(255, 255, 255));
        cv::line(cv_ptr->image, cvPoint(cv_ptr->image.cols >> 1, 0), cvPoint(cv_ptr->image.cols >> 1, cv_ptr->image.rows), CV_RGB(255, 255, 255));

        // draw a box around the DETECTED Qr-Code (!!)
        cv::line(cv_ptr->image, cvPoint(qrLoc[0].x, qrLoc[0].y), cvPoint(qrLoc[1].x, qrLoc[1].y), CV_RGB(0, 0, 0));
        cv::line(cv_ptr->image, cvPoint(qrLoc[0].x, qrLoc[0].y), cvPoint(qrLoc[3].x, qrLoc[3].y), CV_RGB(0, 0, 0));
        cv::line(cv_ptr->image, cvPoint(qrLoc[2].x, qrLoc[2].y), cvPoint(qrLoc[1].x, qrLoc[1].y), CV_RGB(0, 0, 0));
        cv::line(cv_ptr->image, cvPoint(qrLoc[2].x, qrLoc[2].y), cvPoint(qrLoc[3].x, qrLoc[3].y), CV_RGB(0, 0, 0));

        cv::circle(cv_ptr->image, cvPoint(qrLoc[0].x, qrLoc[0].y), 3, CV_RGB(255, 255, 255));
        cv::circle(cv_ptr->image, cvPoint(qrLoc[1].x, qrLoc[1].y), 3, CV_RGB(255, 255, 255));
        cv::circle(cv_ptr->image, cvPoint(qrLoc[2].x, qrLoc[2].y), 3, CV_RGB(255, 255, 255));
        cv::circle(cv_ptr->image, cvPoint(qrLoc[3].x, qrLoc[3].y), 3, CV_RGB(255, 255, 255));

        ostringstream tmpss;

        tmpss << "d:" << qr->dist_z << " dp:" << qr->dist_z_projected << " a:" << qr->angle;
        cv::putText(cv_ptr->image, tmpss.str(), cvPoint(cv_ptr->image.cols >> 3, cv_ptr->image.rows >> 2), 1, 2, CV_RGB(0, 0, 0), 2);

        // show the image
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);

        // save the image!!!
        tmpss.str(string());
        tmpss.clear();
        tmpss << "./images/qr_image_" << *symbol_count << '_';
        tmpss << *ros_time;
        tmpss << ".jpg";

        try {
            cv::imwrite(tmpss.str(), cv_ptr->image);
            cout << "Saved image file as " << tmpss.str() << '\n';
        }
        catch (const std::runtime_error &ex) {
            cout << "Exception saving image : " << ex.what() << '\n';
        }

        cv::waitKey(3);
    }


};

#endif //QR_RADAR2_QRRADAR_H
