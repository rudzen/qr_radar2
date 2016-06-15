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
// Created by rudz on 6/10/16.
//

#ifndef QR_RADAR2_DATA_H
#define QR_RADAR2_DATA_H

#include "QrRadar.h"

template<class T>
class data {

public:
    data() {
        width = width_top = width_bottom = height = height_left = height_right = 0;
    }

    virtual ~data() { }

    data(T width, T width_top, T width_bottom, T height, T height_left, T height_right, Calculator &c) :
            width(width), width_top(width_top), width_bottom(width_bottom), height(height), height_left(height_left), height_right(height_right) {
        dist_z = smallest(c.distance_z_wall(&width), c.distance_z_wall(&height));
    }

    data(T width_top, T width_bottom, T height_left, T height_right, Calculator &c) : width_top(width_top), width_bottom(width_bottom), height_left(height_left), height_right(height_right) {
        width = c.avg(&width_top, &width_bottom);
        height = c.avg(&height_left, &height_right);
        if (c.wall_mode) {
            dist_z = smallest(c.distance_z_wall(&width), c.distance_z_wall(&height));
        } else {
            dist_z = smallest(c.distance_z_floor(&width), c.distance_z_floor(&height));
        }
        angle = c.angle_a(&height, &width);
        dist_z_projected = c.dist_qr_projected(&height, &width, &dist_z);
        if (height_right >= height_left) {
            dist_z_projected = -dist_z_projected;
        }
        dist_z_cam_wall = c.dist_wall(height, width, dist_z);
    }

    T width;
    T width_top;
    T width_bottom;
    T height;
    T height_left;
    T height_right;
    T dist_z;
    T angle;
    T dist_z_projected;
    T dist_z_cam_wall;
};

template<class T>
ostream &operator<<(ostream &stream, data<T> d) {
    stream << d.dist_z << ' ' << d.angle << ' ' << d.dist_z_projected << ' ' << d.dist_z_cam_wall;
    return stream;
}

typedef data<double> ddata;

#endif //QR_RADAR2_DATA_H
