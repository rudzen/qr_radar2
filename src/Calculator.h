//
// Created by rudz on 6/8/16.
//

#ifndef QR_RADAR2_CALCULATOR_H
#define QR_RADAR2_CALCULATOR_H

#define PI 3.14159265

#include "math.h"


//static const float D = 21.05; // cm
static const float D_big = 21.45; // cm
static const float D_floor = 17.15; // cm
static const int Z = 100; // cm

static const int d_big = 114; // pixels
static const float d_floor = d_big * 0.9f; // pixels

static const float focal_front = d_big * Z / D_big; // focal width for front camera
static const float focal_buttom = d_floor * Z / D_floor; // focal width for buttom camera

#define offset(a, b, c) (a - b) / c;


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

    static double avg(double *__restrict__ first, double *__restrict__ second) {
        return (*first + *second) / 2;
    }

    static double angle_a(double h1, double h2, double c) {
        double A = atan((h1 - h2) / (360 / avg(&h1,&h2) )) * 180 / PI;

        double a = c * cos(A);
        return a;
    }

    static double angle_b(double h1, double h2, double c) {

        double A = atan((h1 - h2) / avg(&h1,&h2)) * 180 / PI;
        double a = c / sin(A);
        return a;
    }


    // Function: Perpendicular Distance of a Point J from line formed by Points L and M; Equation of the line ax+by+c=0
    // Description: Given 3 points, the function derives the line quation of the first two points,
    // calculates and returns the perpendicular distance of the the 3rd point from this line.
    static double lineEquation(CvPoint2D32f L, CvPoint2D32f M, CvPoint2D32f J) {
        const float a = -((M.y - L.y) / (M.x - L.x));
        // Now that we have a from the equation ax + by + c, time to substitute (x,y) by values from the Point J
        return (a * J.x + (1.0 * J.y) + (((M.y - L.y) / (M.x - L.x)) * L.x) - L.y) / sqrt((a * a) + (1.0 * 1.0));
    }

    // Function : Converts pixels to cm
    static double pix_to_cm(double *__restrict__ distance) {
        return D_big / *distance;
    }

    // Function: Converts cm to pixels
    static double cm_to_pix(double * __restrict__ cm) {
        return *cm / D_big;
    }

    // Function: Horizontal offset calculation in cm
    // Description: Calculates the horizontal offset in cm of the QR-Code in comparison with the center of the image is was detected in.
    static double offset_horizontal(float * __restrict__ qr_x, float * __restrict__ img_x, double * __restrict__ cm_pix) {
        return offset(*qr_x, *img_x, *cm_pix);
    }

    // Function: Vertical offset calculation in cm
    // Description: Calculates the vertical offset in cm of the QR-Code in comparison with the center of the image is was detected in.
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
    static double distance_z_wall(double *__restrict__ pix_width) {
        return D_big * focal_front / *pix_width;
    }

    static double distance_z_floor(double *__restrict__ pix_height) {
        return D_floor * focal_buttom / *pix_height;
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
