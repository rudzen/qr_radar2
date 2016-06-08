//
// Created by rudz on 6/8/16.
//

#ifndef QR_RADAR2_CALCULATOR_H
#define QR_RADAR2_CALCULATOR_H

#include "PointData.h"


static const float D = 21.45; // cm
static const int Z = 100; // cm
static const float d = 0.0; // pixels
static const float f = 0.0; // focal width

// Class: Mini class wrapping for simple X,Y coordinates.
class Calculator {

private:

public:

    // Function: Routine to get Distance between two points
    // Description: Given 2 points, the function returns the distance
    static double distance(CvPoint2D32f P, CvPoint2D32f Q) {
        //sqrt(pow(abs(P.x - Q.x), 2) + pow(abs(P.y - Q.y), 2));
        return sqrt(abs((int) ((Q.x - P.x) * (Q.x - P.x) + (Q.y - P.y) * (Q.y - P.y))));
    }

    // Function: Perpendicular Distance of a Point J from line formed by Points L and M; Equation of the line ax+by+c=0
    // Description: Given 3 points, the function derives the line quation of the first two points,
    // calculates and returns the perpendicular distance of the the 3rd point from this line.
    static double lineEquation(CvPoint2D32f L, CvPoint2D32f M, CvPoint2D32f J) {
        const float a = -((M.y - L.y) / (M.x - L.x));
        // Now that we have a from the equation ax + by + c, time to substitute (x,y) by values from the Point J
        return (a * J.x + (1.0 * J.y) + (((M.y - L.y) / (M.x - L.x)) * L.x) - L.y) / sqrt((a * a) + (1.0 * 1.0));
    }

    static double pixToCm(double * __restrict__ distance) {
        return D / *distance;
    }

    static double cm_to_pix(double * __restrict__ cm) {
        return *cm / D;
    }

    static double offset_horizontal(float * __restrict__ qr_x, float * __restrict__ img_x, double * __restrict__ cm_pix) {
        return (*qr_x - *img_x) * *cm_pix;
    }

    static double offset_vertical(float * __restrict__ qr_y, float * __restrict__ img_y, double * __restrict__ cm_pix) {
        return (*qr_y - *img_y) * *cm_pix;
    }


    // Function : Calculates the REAL distance in Z axis from an object.
    // Description : The formula REQUIRES several things before it is possible.
    // It requires the real width of the object in cm (D), and by a fixed distance (fx. 100cm) (Z)
    // the width in pixels (d). This is the basis for the focal width (f), which is calculated by :
    // f = d * Z / D.
    // THEN!!!...
    // By having the aparent width of the object in pixels, it is then possible to extrabulate,
    // (a close-enough-but-not-milimeter-acurate) distance.
    // The current distance in cm (Z') divided by the objects real width in cm (D) is equal to
    // the focal width (f) divided by the aparent width of the object in pixels (d').
    // f / d' = Z' / D <-> Z' = D * f / d'
    // ------------------------------------
    // D = cm size of object
    // Z = 100cm
    // d = pixel width at Z distance
    // f = focal width == f=d*Z/D
    // ---------------------------
    // d' = aparent width in pixels at any given distance
    // since f / d' = Z'/ D <-> Z' = D * f / d'
    // Z' is then the actual distance of the CURRENT object detected.
    static double distance_z(float * __restrict__ pix_width) {
        return D * f / *pix_width;
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
