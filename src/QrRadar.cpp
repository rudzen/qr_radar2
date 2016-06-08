//
// Created by rudz on 6/7/16.
//

#include "QrRadar.h"
#include "ros/ros.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "qr_radar");
    QrRadar qr;
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        // Måske bruge ros::spin() i stdet? NEJ!!!!
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}


