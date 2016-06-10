//
// Created by rudz on 6/10/16.
//

#ifndef QR_RADAR2_RECTANGLE_H
#define QR_RADAR2_RECTANGLE_H

template<class T>
struct rect {
    rect() : left(), top(), right(), bottom() {}
    rect(T left, T top, T right, T bottom) :
            left(left), top(top), right(right), bottom(bottom) {}
    template<class Point>
    rect(Point p, T width, T height) :
            left(p.x), right(p.y), right(p.x + width), bottom(p.y + height) {}

    T left;
    T top;
    T right;
    T bottom;

    rect operator+(const rect& that) {
        return rect(left + that.left, top + that.top, right + that.right, bottom + that.bottom);
    }

    int operator<(const rect& that) {
        return left > that.left && right < that.right && top > that.top && bottom < that.bottom;
    }

    int operator>(const rect& that) {
        return left < that.left && right > that.right && top < that.top && bottom > that.bottom;
    }

};

typedef rect<int> intrect;
typedef rect<float> floatrect;

template<class T>
ostream &operator << (ostream &stream, rect<T> r) {
    cout << '[' << r.left << ',' << r.right << ',' << r.top << ',' << r.bottom << ']';
    return stream;
}

#endif //QR_RADAR2_RECTANGLE_H
