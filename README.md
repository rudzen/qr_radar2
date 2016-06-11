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

# FORMAT #

The data being published is send in the following format seperated by ~

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