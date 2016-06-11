//
// Created by rudz on 6/11/16.
//

#ifndef QR_RADAR2_MAP_H
#define QR_RADAR2_MAP_H

// tes

class map_info {

public:

    map_info() { }

    map_info(int pos) : pos(pos) { }

    bool visited;
    bool scanned;
    int pos;
};

class mapqr {

private:

public:

/*
 * W0X.0X/DISTANCE IN M
 *
 * Distance is :
 * n = n - n + 1
 */
    std::map<std::string, map_info> qrmap;

    mapqr() {
        reset();
    }

    void reset() {
        // west wall (blackboard)
        qrmap["W00.00/0"] = map_info(0);
        qrmap["W00.01/0"] = map_info(1);
        qrmap["W00.02/0"] = map_info(2);
        qrmap["W00.03/0"] = map_info(3);
        qrmap["W00.04/0"] = map_info(4);

        qrmap["W00.00/1"] = map_info(5);
        qrmap["W00.01/1"] = map_info(6);
        qrmap["W00.02/1"] = map_info(7);
        qrmap["W00.03/1"] = map_info(8);
        qrmap["W00.04/1"] = map_info(9);

        qrmap["W00.00/2"] = map_info(10);
        qrmap["W00.01/2"] = map_info(11);
        qrmap["W00.02/2"] = map_info(12);
        qrmap["W00.03/2"] = map_info(13);
        qrmap["W00.04/2"] = map_info(14);

        qrmap["W00.00/3"] = map_info(10);
        qrmap["W00.01/3"] = map_info(11);
        qrmap["W00.02/3"] = map_info(12);
        qrmap["W00.03/3"] = map_info(13);
        qrmap["W00.04/3"] = map_info(14);

        // north wall (pointing towards 020)
        qrmap["W01.00/0"] = map_info(4);
        qrmap["W01.01/0"] = map_info(9);
        qrmap["W01.02/0"] = map_info(14);
        qrmap["W01.03/0"] = map_info(19);
        qrmap["W01.04/0"] = map_info(24);

        qrmap["W01.00/1"] = map_info(3);
        qrmap["W01.01/1"] = map_info(8);
        qrmap["W01.02/1"] = map_info(13);
        qrmap["W01.03/1"] = map_info(18);
        qrmap["W01.04/1"] = map_info(23);

        qrmap["W01.00/2"] = map_info(2);
        qrmap["W01.01/2"] = map_info(7);
        qrmap["W01.02/2"] = map_info(12);
        qrmap["W01.03/2"] = map_info(17);
        qrmap["W01.04/2"] = map_info(22);

        qrmap["W01.00/3"] = map_info(1);
        qrmap["W01.01/3"] = map_info(6);
        qrmap["W01.02/3"] = map_info(11);
        qrmap["W01.03/3"] = map_info(16);
        qrmap["W01.04/3"] = map_info(21);

        // east wall (with the "hole" in it)
        qrmap["W02.00/0"] = map_info(24);
        qrmap["W02.01/0"] = map_info(23);
        qrmap["W02.02/0"] = map_info(22);
        qrmap["W02.03/0"] = map_info(21);
        qrmap["W02.04/0"] = map_info(20);

        qrmap["W02.00/1"] = map_info(19);
        qrmap["W02.01/1"] = map_info(18);
        qrmap["W02.02/1"] = map_info(17);
        qrmap["W02.03/1"] = map_info(16);
        qrmap["W02.04/1"] = map_info(15);

        qrmap["W02.00/2"] = map_info(14);
        qrmap["W02.01/2"] = map_info(13);
        qrmap["W02.02/2"] = map_info(12);
        qrmap["W02.03/2"] = map_info(11);
        qrmap["W02.04/2"] = map_info(10);

        qrmap["W02.00/3"] = map_info(9);
        qrmap["W02.01/3"] = map_info(8);
        qrmap["W02.02/3"] = map_info(7);
        qrmap["W02.03/3"] = map_info(6);
        qrmap["W02.04/3"] = map_info(5);

        // south wall (towards 341)
        qrmap["W03.00/0"] = map_info(20);
        qrmap["W03.01/0"] = map_info(15);
        qrmap["W03.02/0"] = map_info(10);
        qrmap["W03.03/0"] = map_info(5);
        qrmap["W03.04/0"] = map_info(0);

        qrmap["W03.00/1"] = map_info(21);
        qrmap["W03.01/1"] = map_info(16);
        qrmap["W03.02/1"] = map_info(11);
        qrmap["W03.03/1"] = map_info(6);
        qrmap["W03.04/1"] = map_info(1);

        qrmap["W03.00/2"] = map_info(22);
        qrmap["W03.01/2"] = map_info(17);
        qrmap["W03.02/2"] = map_info(12);
        qrmap["W03.03/2"] = map_info(7);
        qrmap["W03.04/2"] = map_info(2);

        qrmap["W03.00/3"] = map_info(23);
        qrmap["W03.01/3"] = map_info(18);
        qrmap["W03.02/3"] = map_info(13);
        qrmap["W03.03/3"] = map_info(8);
        qrmap["W03.04/3"] = map_info(3);
    }

    void set_visited(string *qrtext, double * __restrict__ distance) {
        ostringstream ss;
        ss << *qrtext;
        ss << "/";
        ss << (int) *distance;
        cout << ss.str() << endl;
        qrmap.at(ss.str()).visited = true;
    }

    void set_scanned(string *qrtext, ddata *qr_data) {
        ostringstream ss;
        if (qr_data->angle != 0) {

        }
    }

    int get_location(string *qrtext, double * __restrict__ distance) {
        ostringstream key;
        key << *qrtext;
        key << "/";
        key << (int) *distance;
        return qrmap.at(key.str()).pos;
    }


};

/*
ostream &operator << (ostream &stream, map m) {
    for (std::map<string,int>::iterator it=m.begin(); it!=mymap.end(); ++it)
        std::cout << it->first << " => " << it->second << '\n';

    cout << d.dist_z << '~' << d.angle << '~' << d.dist_z_projected << '~' << d.dist_z_cam_wall;
    return stream;
}
*/


#endif //QR_RADAR2_MAP_H
