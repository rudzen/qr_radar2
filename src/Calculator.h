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

    const uint64_t numerator = (1LL << 32) / 1000000;

    //const string qr_codes[] = {"WSunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

    const float D_big = 21.45; // cm
    const float D_floor = 16.65; // cm
    const int Z = 100; // cm

    const int d_big = 114; // pixels
    const float d_floor = d_big * D_floor / D_floor; // pixels

    const float focal_front = d_big * Z / D_big; // focal width for front camera
    const float focal_buttom = d_floor * Z / D_floor; // focal width for buttom camera

    // room size
    v3<float> room = v3<float>(960, 1075, 340);
    const float north_south_interval = room.y / 6; // 5 codes each wall, so 6 spaces.
    const float east_west_interval = room.x / 6;

public:
    const v3<float> &getRoom() const {
        return room;
    }

    const double getWallBehind(char *c, double * __restrict__ forward_distance) {
        switch (*c) {
            case 0:
            case 2:
                return *forward_distance - room.y;
            case 1:
            case 3:
                return *forward_distance - room.x;
            default:break;
        }
    }

    float getLeftWallDistance(string *qr_text, double * __restrict__ qr_offset) {
        return qr_wall_dist[*qr_text].first + (float) *qr_offset;
    }

    float getRightWallDistance(string *qr_text, double * __restrict__ qr_offset) {
        return qr_wall_dist[*qr_text].second + (float) *qr_offset;
    }

private:
    typedef pair<float, float> wall_distance;
    map<string, wall_distance> qr_wall_dist;

    void set_qr_distances() {

        // unrolled loops (for speed!!!)

        // "N" (white boards)
        qr_wall_dist["W00.00"] = wall_distance(north_south_interval, north_south_interval * 5);
        qr_wall_dist["W00.01"] = wall_distance(north_south_interval * 2, north_south_interval * 4);
        qr_wall_dist["W00.02"] = wall_distance(north_south_interval * 3, north_south_interval * 3);
        qr_wall_dist["W00.03"] = wall_distance(north_south_interval * 4, north_south_interval * 2);
        qr_wall_dist["W00.04"] = wall_distance(north_south_interval * 5, north_south_interval);

        // S
        qr_wall_dist["W02.00"] = qr_wall_dist["W00.00"]; //wall_distance(north_south_interval, north_south_interval * 5);
        qr_wall_dist["W02.01"] = qr_wall_dist["W00.01"]; //wall_distance(north_south_interval * 2, north_south_interval * 4);
        qr_wall_dist["W02.02"] = qr_wall_dist["W00.02"]; //wall_distance(north_south_interval * 3, north_south_interval * 3);
        qr_wall_dist["W02.03"] = qr_wall_dist["W00.03"]; //wall_distance(north_south_interval * 4, north_south_interval * 2);
        qr_wall_dist["W02.04"] = qr_wall_dist["W00.04"]; //wall_distance(north_south_interval * 5, north_south_interval);

        // E
        qr_wall_dist["W01.00"] = wall_distance(east_west_interval, east_west_interval * 5);
        qr_wall_dist["W01.01"] = wall_distance(east_west_interval * 2, east_west_interval * 4);
        qr_wall_dist["W01.02"] = wall_distance(east_west_interval * 3, east_west_interval * 3);
        qr_wall_dist["W01.03"] = wall_distance(east_west_interval * 4, east_west_interval * 2);
        qr_wall_dist["W01.04"] = wall_distance(east_west_interval * 5, east_west_interval);

        // W
        qr_wall_dist["W03.00"] = qr_wall_dist["W01.00"]; //wall_distance(east_west_interval, east_west_interval * 5);
        qr_wall_dist["W03.01"] = qr_wall_dist["W01.00"]; //wall_distance(east_west_interval * 2, east_west_interval * 4);
        qr_wall_dist["W03.02"] = qr_wall_dist["W01.00"]; //wall_distance(east_west_interval * 3, east_west_interval * 3);
        qr_wall_dist["W03.03"] = qr_wall_dist["W01.00"]; //wall_distance(east_west_interval * 4, east_west_interval * 2);
        qr_wall_dist["W03.04"] = qr_wall_dist["W01.00"]; //wall_distance(east_west_interval * 5, east_west_interval);

    }

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

    uint64_t nanoToMili(uint32_t time) {
        return (time * numerator) >> 32;
    }

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




};

#endif //QR_RADAR2_CALCULATOR_H
