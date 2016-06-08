//
// Created by rudz on 6/8/16.
//

#ifndef QR_RADAR2_POINTDATA_H
#define QR_RADAR2_POINTDATA_H


// Class: Mini class wrapping for simple X,Y coordinates.
class PointData {
public:
    int x;
    int y;
    PointData(int x_, int y_) {
        x = x_;
        y = y_;
    }
};



#endif //QR_RADAR2_POINTDATA_H
