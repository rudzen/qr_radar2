## README ##

Designed for the course 02343 CDIO-Projekt F16, DTU.

Basic QR code scanning of topic `image`. This is achieved by using the zbar (http://zbar.sourceforge.net/) barcode reader library.
Published the following information from detected QR-codes to the topic `qr` topic.

* ROS Timestamp
* QR-Code text
* QR-Code horizontal offset (cm)
* QR-Code vertical offset (cm)
* Distance to QR-Code in CM

# FORMAT #

The data being published is send in the following format.

ROSTIME~QRTEXT~H.OFFSET~V.OFFSET~CM

Note that the offsets are ZERO PADDED if needed.
The format is padded to allow for a maximum of 99999 value for each, meaning that any values which essentially is fewer characters long will be padded.

For example if a QR-code containing the text P01 and has the offsets at 5 cm (h) and 4 cm (v) from a distance of 1,02 m:

567567322.P01.00005.00004.00102

(Might be altered depending on the needs of the map node. One options could be to calculate the distance from the center alignment of the current image in both X and Y axis and attach that information as well.)

# WHY? #
The main reason for the coordinates is to identify any discrepancy in alignment if

* Hovering over a landing field and the alignment is crucial to landing correctly. This might be because the landing fields are not directly located the same locations as the drone mapping center points.
* Detecting distance from any wall which contains a QR-code. The size of the QR code gives a hint to how close the drone is to the actual wall. This is possible because the "real" size of the QR-code is known before hand.
* Align X- and Y-axis of the QR-Code in the center of the drones camera.
* Align for flying through a circular opening marked with a QR-Code.