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
// Created by rudz on 6/9/16.
//
#pragma clang diagnostic push
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"
#ifndef QR_RADAR2_VEC_H
#define QR_RADAR2_VEC_H

using namespace std;

template<class T>
class v2 {
public:

    v2(T x1, T y1, T x2, T y2) {
        x = x2 - x1;
        y = y2 - y1;
    }

    v2(T x_, T y_) {
        x = x_;
        y = y_;
    }

    v2() {
        x = 0;
        y = 0;
    }

    T x;
    T y;

    v2 operator+(const v2& that) {
        return v2(x + that.x, y + that.y);
    }

    v2 operator-(const v2& that) {
        return v2(x - that.x, y - that.y);
    }

    // Operator : Scalarproduct (dotproduct)
    T operator*(const v2& that) {
        return (x * that.x) + (y * that.y);
    }

    v2 operator*(T * __restrict__ k) {
        return v2(*k * x, *k * y);
    }

    int operator/(const v2& that) {
        return v2(x, y) * that == 0 ? 1 : 0;
    }

    v2 project_onto(const v2& that) {
        return this * (v2(x, y) * that / pow(len(), 2));
    }

    T project_len(const v2& that) {
        return abs(v2(x, y) * that / len());
    }

    T angle(const v2& that) {
        return (v2(x, y) * that) / (len() * that.len());
    }

    T len()const {
        return abs(sqrt((x * x) + (y * y)));
    }

    v2 cross() {
        return v2(-y, x);
    }

    T det(const v2& that) {
        return cross() * that;
    }

    T parallel(const v2& that) {
        return det(that) == 0 ? 1 : 0;
    }

};

template<class T>
ostream &operator << (ostream &stream, v2<T> v) {
    cout << '[' << v.x << ',' << ']';
    return stream;
}

template <class T>
class v3 : public v2<T> {
public:
    v3(T x_, T y_, T z_): v2<T>(x_, y_) {
        z = z_;
    }

    v3(): v2<T>(0, 0) {
        z = 0;
    }

    T z;

    v3 operator+(const v3& that) {

    }

    v3 operator-(const v3& that) {

    }

    v3 operator*(const v3& that) {

    }

    T len() {

    }


};


#endif //QR_RADAR2_VEC_H

#pragma clang diagnostic pop