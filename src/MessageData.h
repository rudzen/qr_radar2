//
// Created by rudz on 6/14/16.
//

#ifndef QR_RADAR2_MESSAGEDATA_H
#define QR_RADAR2_MESSAGEDATA_H

#include <string>

using namespace std;

class message_data {

private:

    const char delim = '\n';

    vector<string> &split(const string &s, vector<string> &elems) {
        stringstream ss(s);
        string item;
        while (std::getline(ss, item, delim)) {
            elems.push_back(item);
        }
        return elems;
    }

    vector<string> split(const string &s) {
        vector<string> elems;
        split(s, elems);
        return elems;
    }

public:
    typedef pair<double, double> offsets;

    message_data(double ros_time, const string &text, double offset_x, double offset_y, double dist_z, double angle, double dist_wall, const pair<double, double> &offsets) : ros_time(ros_time), text(text),
                                                                                                                                                               dist_z(dist_z), angle(angle),
                                                                                                                                                               dist_wall(dist_wall)
                                                                                                                                                                {
                                                                                                                                                                    xy.first = offset_x;
                                                                                                                                                                    xy.second = offset_y;
                                                                                                                                                                }

    message_data(const string &qrdata) {
        vector<string> dat = split(qrdata);
        ros_time = atof(dat[0].c_str());
        text = dat[1].c_str();
        xy.first = atof(dat[2].c_str());
        xy.second = atof(dat[3].c_str());
        dist_z = atof(dat[4].c_str());
        angle = atof(dat[5].c_str());
        dist_wall = atof(dat[6].c_str());
    }

    double ros_time;
    std::string text;
    offsets xy;
    double dist_z;
    double angle;
    double dist_wall;
};

ostream &operator<<(ostream &stream, message_data d) {
    stream << d.ros_time << '\n' << d.text << '\n' << d.xy.first << '\n' << d.xy.second << '\n' << d.dist_z << '\n' << d.angle << '\n' << d.dist_wall;
    return stream;
}

#endif //QR_RADAR2_MESSAGEDATA_H
