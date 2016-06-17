//
// Created by rudz on 6/17/16.
//

#ifndef QR_RADAR2_ZB_H
#define QR_RADAR2_ZB_H

#include <zbar.h>

template <class T>
class zb {

private:

    zbar::ImageScanner is;
    zbar::Image img;

    T size;

public:

    zbar::ImageScanner *imageScanner;
    zbar::Image *img;

    ZBar() {
        zbar::zbar_scanner_create(&is);
        *imageScanner = &is;
    }

};


#endif //QR_RADAR2_ZB_H
