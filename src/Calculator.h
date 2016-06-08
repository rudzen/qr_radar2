//
// Created by rudz on 6/8/16.
//

#ifndef QR_RADAR2_CALCULATOR_H
#define QR_RADAR2_CALCULATOR_H

#include "PointData.h"


static const float qr_size = 21.45; // cm

// Class: Mini class wrapping for simple X,Y coordinates.
class Calculator {

private:

public:

    // Function: Routine to get Distance between two points
    // Description: Given 2 points, the function returns the distance
    static double distance(CvPoint2D32f P, CvPoint2D32f Q) {
        return sqrt(abs((int) ((Q.x - P.x) * (Q.x - P.x) + (Q.y - P.y) * (Q.y - P.y))));
        //return sqrt(pow(abs(P.x - Q.x), 2) + pow(abs(P.y - Q.y), 2));
    }

    // Function: Perpendicular Distance of a Point J from line formed by Points L and M; Equation of the line ax+by+c=0
    // Description: Given 3 points, the function derives the line quation of the first two points,
    //	  calculates and returns the perpendicular distance of the the 3rd point from this line.
    static double lineEquation(CvPoint2D32f L, CvPoint2D32f M, CvPoint2D32f J) {
        const float a = -((M.y - L.y) / (M.x - L.x));
        // Now that we have a from the equation ax + by + c, time to substitute (x,y) by values from the Point J
        return (a * J.x + (1.0 * J.y) + (((M.y - L.y) / (M.x - L.x)) * L.x) - L.y) / sqrt((a * a) + (1.0 * 1.0));
    }

    static double pixToCm(double * __restrict__ distance) {
        return qr_size / *distance;
    }

    static double cm_to_pix(double * __restrict__ cm) {
        return *cm / qr_size;
    }

    /*
     *
     *
     *
     calculation of offset :
     float x = 0, y = 0;
        for (final PointF p : AnalysisData.hits) {
            x += p.x;
            y += p.y;
        }
        swarmCenter = new PointF(x / AnalysisData.hitCount, y / AnalysisData.hitCount);

        Log.d("swarmcenter", swarmCenter.toString());

        offSetHorizontal = (poa.x - swarmCenter.x) * cmPrPix;
        offSetVertical = (poa.y - swarmCenter.y) * cmPrPix;

        offSetHorizontalDirection = offSetHorizontal > 0 ? LEFT : RIGHT;
        offSetVerticalDirection = offSetVertical > 0 ? UP : DOWN;

     *
     */
};




#endif //QR_RADAR2_CALCULATOR_H
