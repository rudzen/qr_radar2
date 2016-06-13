## README ##

Designed for the course 02343 CDIO-Projekt F16, DTU.

Basic QR code scanning of topic `image`. This is achieved by using the zbar (http://zbar.sourceforge.net/) barcode reader library.
Published the following information from detected QR-codes to the topic `qr` topic.

* ROS Time stamp
* QR-Code text
* QR-Code horizontal offset (cm)
* QR-Code vertical offset (cm)
* Distance to QR-Code in CM
* Projected distance to perpendicular distance from QR-Code based on camera position
* Angle of QR-code scanned based on camera angle


# INSTALL #

```
roscd
cd catkin_ws/src
git clone https://bitbucket.org/rudz/qr_radar2.git
cd ..
catkin_make
```

Then, input these lines into the droneScript.sh

```
gnome-terminal --tab --working-directory=/opt/ros/indigo/catkin_ws/ -e '/bin/bash -c "pwd; source devel/setup.bash; cd src/qr_radar2/; rosrun qr_radar2 qr_radar2;"'
sleep 3
```

# TOPICS (what is what) #

## Publishing ##

```
/qr/ (std_msgs/String) - The main publishing topic for discovered qr codes, and control change responses.
```

## Subscriptions ##


```
/ardrone/image_raw (image_transport) - The main subscription where it receives it's images to scan.
qr/scan/topic (std_msgs/String) - Changes the image subscription topic.
qr/scan/wall (std_msgs/Empty) - Flips between wall and floor sized QR-code measurements to achieve higher accuracy in distances.
qr/scan/set (std_msgs/Bool) - manually dis-/enables the scanning of received images.
qr/throttle (std_msgs/Float32) - Control the throttle through here. Min. allowed is > 0
qr/control (std_msgs/Byte) - Control the scanning areas using this.
qr/display/enable (std_msgs/Empty) - Enables the output image display of discovered QR-codes.
qr/display/disable (std_msgs/Empty) - Disables the output image display of discovered QR-codes.
qr/display/set (std_msgs/Bool) - Manually changes the display window on or off.
```






# FORMAT #

The data being published is send in the following format seperated by \n

* ROSTIME (uint_32)
* QRTEXT (string)
* HORIZONTAL OFFSET IN VIEW (CM) (double)
* VERTICAL OFFSET IN VIEW (CM) (double)
* REAL_DIRECT_DISTANCE (CM) (double)
* ANGLE (DEG) (double)
* PROJECTED PERPENDICULAR DISTANCE FROM QR-CODE (CM) (double)
* DIRECT SHORTEST DISTANCE FROM CAMERA TO WALL (CM) (double)

# WHY? #
The main reason for the coordinates is to identify any discrepancy in alignment if

* Hovering over a landing field and the alignment is crucial to landing correctly. This might be because the landing fields are not directly located the same locations as the drone mapping center points.
* Detecting distance from any wall which contains a QR-code. The size of the QR code gives a hint to how close the drone is to the actual wall. This is possible because the "real" size of the QR-code is known before hand.
* Align X- and Y-axis of the QR-Code in the center of the drones camera.
* Align for flying through a circular opening marked with a QR-Code.