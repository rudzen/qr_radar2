/*
 * The MIT License
 *
 * Copyright 2016 Rudy Alex Kohn (s133235@student.dtu.dk).
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
//
// Created by rudz on 6/8/16.
//

#ifndef QR_RADAR2_CALCULATOR_H
#define QR_RADAR2_CALCULATOR_H

#include "math.h"
#include "Vec.h"
#include "Rectangle.h"
#include "ControlHeaders.h"

#define smallest(a, b) (a < b ? a : b)
#define largest(a, b) (a < b ? b : a)

// Class: Mini class wrapping for simple X,Y coordinates.
class Calculator {

private:

    const float D_big = 21.45; // cm
    const float D_floor = 16.65; // cm
    const int Z = 100; // cm

    const int d_big = 114; // pixels
    const float d_floor = d_big * D_floor / D_floor; // pixels

    const float focal_front = d_big * Z / D_big; // focal width for front camera
    const float focal_buttom = d_floor * Z / D_floor; // focal width for buttom camera

public:

    bool wall_mode; // default is wall_mode

    Calculator() {
        wall_mode = true;
    }

    Calculator(bool wall_mode) : wall_mode(wall_mode) { }

// Function: Routine to get Distance between two points
    // Description: Given 2 points, the function returns the distance
    int pixel_distance(const v2<int> &P, const v2<int> &Q) {
        //sqrt(pow(abs(P.x - Q.x), 2) + pow(abs(P.y - Q.y), 2));
        return sqrt(abs(((Q.x - P.x) * (Q.x - P.x) + (Q.y - P.y) * (Q.y - P.y))));
    }

    int pytha(int *__restrict__ a, int * __restrict__ b) {
        return (int) sqrt(*a * *a + *b * *b);
    }

    double avg(double *__restrict__ first, double *__restrict__ second) {
        return (*first + *second) / 2;
    }

    double angle_a(double h, double w) {
        const double a = w / h;
        if (a > 1 || a < -1) return 0;
        // convert from radians to degrees :
        return acos(a) * 180 / M_PI;
    }

    double angle_a(double *__restrict__ h, double *__restrict__ w) {
        const double a = *w / *h;
        if (a > 1 || a < -1) return 0;
        // convert from radians to degrees :
        return acos(a) * 180 / M_PI;
    }

    double dist_qr_projected(double h, double w, double dist, int modifier) {
        const double a = w / h;
        if (a > 1 || a < -1) return dist;
        const double A = acos(a);
        return sin(A) * dist * modifier;
    }

    double dist_qr_projected(double *__restrict__ h, double *__restrict__ w, double *__restrict__ dist) {
        const double a = *w / *h;
        if (a > 1 || a < -1) return *dist;
        const double A = acos(a);
        return sin(A) * *dist;
    }

    double dist_wall(double h, double w, double dist) {
        const double a = w / h;
        if (a > 1 || a < -1) return dist;
        // convert from radians to degrees :
        const double A = acos(a);// * 180 / M_PI;
        return cos(A) * dist;
    }

    double angle_floor(double distance, double width, double height) {
        return atan((width / 2) / distance);
    }

    // Function: Perpendicular Distance of a Point J from line formed by Points L and M; Equation of the line ax+by+c=0
    // Description: Given 3 points, the function derives the line quation of the first two points,
    // calculates and returns the perpendicular distance of the the 3rd point from this line.
    /*
    static double lineEquation(v2 L, v2 M, v2 J) {
        int my_minus_ly = M.y - L.y;
        int mx_minus_lx = M.x - L.x;
        const double a = -((my_minus_ly) / (mx_minus_lx));
        // Now that we have a from the equation ax + by + c, time to substitute (x,y) by values from the Point J
        return (a * J.x + (1.0 * J.y) + (((my_minus_ly) / (mx_minus_lx)) * L.x) - L.y) / sqrt((a * a) + 1.0);
    }
     */

    // Function : Converts pixels to cm
    double pix_to_cm(int *__restrict__ width) {
        return D_big / *width;
    }

    double pix_to_cm(int width) {
        return (wall_mode ? D_big : D_floor) / width;
    }

    // Function: Converts cm to pixels
    int cm_to_pix(double *__restrict__ cm) {
        return (int) round(*cm / (wall_mode ? D_big : D_floor));
    }

    // Function: Horizontal offset calculation in cm
    // Description: Calculates the horizontal offset in cm of the QR-Code in comparison with the center of the image is was detected in.
    double offset_horizontal(int *__restrict__ qr_x, int *__restrict__ img_x, double *__restrict__ cm_pix) {
        // > 0 = left, < 0 right
        return (*qr_x - *img_x) * *cm_pix;
    }

    // Function: Vertical offset calculation in cm
    // Description: Calculates the vertical offset in cm of the QR-Code in comparison with the center of the image is was detected in.
    double offset_vertical(int *__restrict__ qr_y, int *__restrict__ img_y, double *__restrict__ cm_pix) {
        // > 0 up, < 0 down
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
    // f = focal width == f = d * Z / D
    // ---------------------------
    // d' = aparent width in pixels at any given distance
    // since f / d' = Z'/ D <-> Z' = D * f / d'
    // Z' is then the actual distance of the CURRENT object detected.
    double distance_z_wall(double *__restrict__ pix_width) {
        return D_big * focal_front / *pix_width;
    }

    double distance_z_floor(double *__restrict__ pix_width) {
        return D_floor * focal_buttom / *pix_width;
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

    //Function : Get scanning image dimensions based on the current control setting
    //Description : Depending on the setting, the rectangular configuration is set
    //              to reflect the current location where the scanner is to look.
    intrect static get_img_dim(int *__restrict__ control, int cols, int rows) {
        intrect ret;
        switch (*control) {
            case QR_CONTROL_ALL:
                ret.top = 0;
                ret.bottom = rows;
                ret.left = 0;
                ret.right = cols;
                break;
            case QR_CONTROL_LEFT:
                ret.top = 0;
                ret.bottom = rows;
                ret.left = 0;
                ret.right = cols >> 1;
                break;
            case QR_CONTROL_RIGHT:
                ret.top = 0;
                ret.bottom = rows;
                ret.left = cols >> 1;
                ret.right = cols;
                break;
            case QR_CONTROL_UPPER:
                ret.top = 0;
                ret.bottom = rows >> 1;
                ret.left = 0;
                ret.right = cols;
                break;
            case QR_CONTROL_LOWER:
                ret.top = rows >> 1;
                ret.bottom = rows;
                ret.left = 0;
                ret.right = cols;
                break;
            case QR_CONTROL_QUAD_1:
                ret.top = rows;
                ret.bottom = rows >> 1;
                ret.left = cols >> 1;
                ret.right = rows;
                break;
            case QR_CONTROL_QUAD_2:
                ret.top = rows;
                ret.bottom = rows >> 1;
                ret.left = 0;
                ret.right = cols >> 1;
                break;
            case QR_CONTROL_QUAD_3:
                ret.top = rows >> 1;
                ret.bottom = rows;
                ret.left = 0;
                ret.right = cols >> 1;
                break;
            case QR_CONTROL_QUAD_4:
                ret.top = rows >> 1;
                ret.bottom = rows;
                ret.left = cols >> 1;
                ret.right = cols;
                // yep, fallthrough...
            default:
                break;
        }
        return ret;
    }

    void balance_white(cv::Mat mat) {
        double discard_ratio = 0.05;
        int hists[3][256];
        memset(hists, 0, 3 * 256 * sizeof(int));

        for (int y = 0; y < mat.rows; ++y) {
            uchar *ptr = mat.ptr<uchar>(y);
            for (int x = 0; x < mat.cols; ++x) {
                for (int j = 0; j < 3; ++j) {
                    hists[j][ptr[x * 3 + j]] += 1;
                }
            }
        }

        // cumulative hist
        int total = mat.cols * mat.rows;
        int vmin[3], vmax[3];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 255; ++j) {
                hists[i][j + 1] += hists[i][j];
            }
            vmin[i] = 0;
            vmax[i] = 255;
            while (hists[i][vmin[i]] < discard_ratio * total)
                vmin[i] += 1;
            while (hists[i][vmax[i]] > (1 - discard_ratio) * total)
                vmax[i] -= 1;
            if (vmax[i] < 255 - 1)
                vmax[i] += 1;
        }


        for (int y = 0; y < mat.rows; ++y) {
            uchar *ptr = mat.ptr<uchar>(y);
            for (int x = 0; x < mat.cols; ++x) {
                for (int j = 0; j < 3; ++j) {
                    int val = ptr[x * 3 + j];
                    if (val < vmin[j])
                        val = vmin[j];
                    if (val > vmax[j])
                        val = vmax[j];
                    ptr[x * 3 + j] = static_cast<uchar>((val - vmin[j]) * 255.0 / (vmax[j] - vmin[j]));
                }
            }
        }
    }


};

#endif //QR_RADAR2_CALCULATOR_H
