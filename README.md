## README ##

Designed for the course 02343 CDIO-Projekt F16, DTU.

Basic QR code scanning of topic `image`. This is achieved by using the zbar (http://zbar.sourceforge.net/) barcode reader library.
Published the following information from detected QR-codes to the topic `qr` topic.

NOTE: This is just a part of a complete project.

#FEATURES#

* Somewhat optimized C++ code.
* Multiple threads to publish vital data which reduces delay.
* ROS Time stamping of processed QR-Code.
* Functions perfectly in almost **any** light conditions (except complete darkness and powerful spotlights).
* Maximum centered distance for reading QR-Code is approx. 4m (with 360p front camera).
* Configurable re-send throttle timer, based on ROS time, (default 2 seconds) to avoid QR-spamming and map configuration overload.
* QR-Code text publishing.
* QR-Code horizontal offset in cm.
* QR-Code vertical offset in cm.
* Distance to QR-Code in cm.
* Projected distance to perpendicular distance from QR-Code based on camera position.
* Angle of QR-code scanned based on camera-wall when QR-Code is read, in degrees.
* Distances to **all** four walls no matter which QR-Code is scanned.
* Wall/Floor mode switching.
* Floor mode optional Drone controlling to center of QR-Code (not finished)
* Set incoming image feed topic.
* Flip between bottom and front camera.
* Dis-/Enable debug window.
* Dis-/Enable scanning of QR-Codes.

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

or, if sourced :
```
rosrun qr_radar2 qr_radar2
```


# TOPICS (what is what) #

## Publishing ##

```
/qr/ (std_msgs/String) - The main publishing topic for discovered qr codes, and control change responses.
/qr/scan/count (std_msgs/String) - Number of QR-Codes found
/collision/wall (std_msgs/String) - If distance is <= 2.0m
```

## Subscriptions ##


```
/ardrone/front/image_raw (image_transport) - (default) The main subscription where it receives it's images to scan.
qr/scan/topic (std_msgs/String) - Changes the image subscription topic.
qr/scan/mode (std_msgs/Empty) - Flips between wall and floor sized QR-code measurements to achieve higher accuracy in distances.
qr/scan/set (std_msgs/Bool) - manually dis-/enables the scanning of received images.
qr/scan/set/wall (std_msgs/String) - Dis-/enable specific wall detection (0-3)

qr/throttle/set (std_msgs/Float32) - Control the throttle through here. Min. allowed is > 0

qr/display/enable (std_msgs/Empty) - Enables the output image display of discovered QR-codes.
qr/display/disable (std_msgs/Empty) - Disables the output image display of discovered QR-codes.
qr/display/set (std_msgs/Bool) - Manually changes the display window on or off.

qr/kaffe (std_msgs/Empty) - Black or with cream?
```

# FORMAT #

The data being published is send in the following format seperated by a single space (' ')

* Rostime (from ros::Time::now.ms) (uint_32)
* Qr Text (string)
* Horizontal offset from camera center (cm) (double)
* Vertical offset from camera center (cm) (double)
* Real direct distance (cm) (double)
* Angle of drone to wall when QR-Code was read (degrees) (double)
* Projected distance (perpendicular) to where the QR-Code would be dead-center in camera (cm) (double)
* Real direct distance from camera to wall where QR-Code is located (cn) (double)
* Distance to wall #0 from drone (cm) (double)
* Distance to wall #1 from drone (cm) (double)
* Distance to wall #2 from drone (cm) (double)
* Distance to wall #3 from drone (cm) (double)
