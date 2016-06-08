//
// Created by rudz on 6/8/16.
//

#ifndef QR_RADAR2_CALCULATOR_H
#define QR_RADAR2_CALCULATOR_H

#include "PointData.h"

// Class: Mini class wrapping for simple X,Y coordinates.
class Calculator {
public:
    // Function: Routine to get Distance between two points
    // Description: Given 2 points, the function returns the distance
    static double cv_distance(PointData P, PointData Q) {
        return sqrt(pow(abs(P.x - Q.x), 2) + pow(abs(P.y - Q.y), 2));
    }

    // Function: Perpendicular Distance of a Point J from line formed by Points L and M; Equation of the line ax+by+c=0
    // Description: Given 3 points, the function derives the line quation of the first two points,
    //	  calculates and returns the perpendicular distance of the the 3rd point from this line.
    static double cv_lineEquation(PointData L, PointData M, PointData J) {
        const float a = -((M.y - L.y) / (M.x - L.x));
        // Now that we have a from the equation ax + by + c, time to substitute (x,y) by values from the Point J
        return (a * J.x + (1.0 * J.y) + (((M.y - L.y) / (M.x - L.x)) * L.x) - L.y) / sqrt((a * a) + (1.0 * 1.0));
    }

};

#endif //QR_RADAR2_CALCULATOR_H
