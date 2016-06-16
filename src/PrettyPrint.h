/*
 * The MIT License
 *
 * Copyright 2015-6 Rudy Alex Kohn (s133235@student.dtu.dk).
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
// Created by rudz on 6/12/16.
//

#ifndef PRETTYPRINT_H
#define PRETTYPRINT_H

#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "ros/ros.h"
#include <string>
#include <vector>
#include <boost/thread.hpp>
#include <algorithm>
#include <functional>
#include <cctype>
#include <locale>


/**
 * Simple class to print out stuff to console that looks nice!
 * @author rudz
 */
class PrettyPrint {

    typedef pair<int, string> menuentry;

    const int MAX_LEN = 78;
    //const int MAX_HEI = 20;

    const char dot = '^';
    //const char data_sep = '|';

    const string SEP = replicate(dot, MAX_LEN);

    const string TITEL = "QR-Radar ROS-node v";
    const string TITEL2 = "<< Gruppe 8 >>";
    const string INFO = "R.A.Kohn";

    //const int TITLE_LEN = (int) (TITEL.length() + TITEL2.length());

    //int port;
    int pos = -1;
    string version;

    vector<menuentry> menu;

private:

    vector<string> &split(const string &s, char delim, vector<string> &elems) {
        stringstream ss(s);
        string item;
        while (std::getline(ss, item, delim)) {
            elems.push_back(item);
        }
        return elems;
    }

    vector<string> split(const string &s, char delim) {
        vector<string> elems;
        split(s, delim, elems);
        return elems;
    }

    // trim from start
    static inline std::string &ltrim(std::string &s) {
        s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>((int (*)(int)) isspace))));
        return s;
    }

    // trim from end
    static inline std::string &rtrim(std::string &s) {
        s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>((int (*)(int)) isspace))).base(), s.end());
        return s;
    }

    // trim from both ends
    static inline std::string &trim(std::string &s) {
        return ltrim(rtrim(s));
    }

    void init() {
        get_top(&pos);
        get_status(&pos);
        get_buttom(&pos);
        menu.shrink_to_fit();
    }

public:

    vector<menuentry> data;

    PrettyPrint(const string &version) : version(version) {
        init();
    }

    PrettyPrint() {
        init();
    }

    /**
     * Prints out the current menu to console.
     */
    void show_menu() {
        for (menuentry &me : menu) {
            cout << me.second << '\n';
        }
        cout << endl;
    }

    /**
     * Generate the top part of all menus
     *
     * @param pos : the current pos in the menu generating process
     */
    void get_top(int *pos) {
        menu.push_back(menuentry(++*pos, SEP));
        ostringstream ss;
        ss << TITEL << version;
        menu.push_back(menuentry(++*pos, makeFilledBorder(ss.str())));
        menu.push_back(menuentry(++*pos, makeFilledBorder(TITEL2)));
        menu.push_back(menuentry(++*pos, SEP));
        menu.push_back(menuentry(++*pos, makeSingleBorderCentered(INFO)));
    }

    /**
     * Generates the status part of the menu screen.
     *
     * @param pos : The current row position.
     * @return the row position when done.
     */
    void get_status(int *pos) {
        ostringstream ss;
        menu.push_back(menuentry(++*pos, makeSingleBorderCentered("s133235@student.dtu")));
        menu.push_back(menuentry(++*pos, makeFilledBorder("Status")));
        ss << " Date/Time build : ";
        std::string data = __DATE__;
        std::string time = __TIME__;
        ss << trim(data) << " / " << trim(time);
        menu.push_back(menuentry(++*pos, makeFilledBorder(ss.str())));
    }

    /**
     * Makes the last bit of the menu screen.
     *
     * @param pos : current row pos
     * @return The current row pos when done.
     */
    void get_buttom(int *pos) {

        /* Not used atm, because this is actually not a menu YET! */
//        while (p < MAX_HEI) {
//            this.menu.add(++p, dot + space(MAX_LEN - 2) + dot);
//        }
        menu.push_back(menuentry(++*pos, overwrite(SEP, " < powered by con-dynmenu v0.3 by rudz > ", 30)));
    }

    /**
     * Encapsulates a specified string with filled "border" of current
     * 'dot'.<br>
     * The parsed string will be centred.
     *
     * @param s : the string to encapsulate
     * @return the completed string
     */
    string makeFilledBorder(string s) {
        unsigned long len = s.length();
        ostringstream ss;
        if (len < MAX_LEN - 2) {
            if (len == MAX_LEN - 4) {
                ss << dot << ' ' << s << ' ' << dot;
            } else {
                unsigned long lenMod2 = len % 2;
                string dots = replicate(dot, (int) (MAX_LEN - lenMod2 - 2 - len >> 1));
                ss << dots;
                ss << ' ' << s << ' ';
                ss << dots;
                if (len % 2 != 0) {
                    ss << replicate(dot, (int) lenMod2);
                }
            }
        } else {
            ss << s;
        }
        return ss.str();
    }

    /**
     * Generates a line containing a single 'dot' at the beginning and end.<b>
     * The rest is filled with spaces and the parsed string is centred.
     *
     * @param s : the string to centre
     * @return : the resulting string
     */
    string makeSingleBorderCentered(string s) {
        unsigned long len = s.length();
        if (len > MAX_LEN - 2) {
            return s;
        }
        ostringstream ss;
        if (len == MAX_LEN - 4) {
            ss << dot << ' ' << s << ' ' << dot;
        } else if (len < MAX_LEN - 4) {
            ss << dot;
            unsigned long lenMod2 = len % 2;
            string dots = space((int) (MAX_LEN - lenMod2 - 4 - len >> 1));
            ss << dots;
            ss << ' ' << s << ' ';
            if (len % 2 > 0) {
                ss << space((int) lenMod2);
            }
            ss << dots << dot;
        } else {
            ss << s;
        }
        return ss.str();
    }

    /**
     * Generates a line containing a single 'dot' at the beginning and end.<b>
     * The string will follow after a space from the left, filled with spaces up
     * until the last 'dot'.
     *
     * @param s : the string to encapsulate.
     * @return : the resulting string
     */
    string makeSingleBordered(string s) {
        unsigned long len = s.length();
        if (len > MAX_LEN - 2) {
            return s;
        }
        ostringstream ss;
        ss << dot << s << space((MAX_LEN - 3 - len)) << dot;
        return ss.str();
    }

    /**
     * Creates a string containing spaces
     *
     * @param amount : how many spaces to make
     * @return the string containing amount spaces
     */
    string space(int amount) {
        return replicate(' ', amount);
    }

    string space(unsigned long amount) {
        return replicate(' ', amount);
    }

    /**
     * Replicate a char
     *
     * @param c : the char to replicate
     * @param amount : how many time to replicate
     * @return the string containing amount char
     */
    string replicate(char c, unsigned long amount) {
        ostringstream ss;
        for (int i = 1; i <= amount; ++i) {
            ss << c;
        }
        return ss.str();
    }

    string replicate(char c, int amount) {
        ostringstream ss;
        for (int i = 1; i <= amount; ++i) {
            ss << c;
        }
        return ss.str();
    }

    string replicate(string c, int amount) {
        ostringstream ss;
        for (int i = 1; i <= amount; ++i) {
            ss << c;
        }
        return ss.str();
    }

    /**
     * Replaces a piece of an existing string with another string.<br>
     *
     * @param into : The string to overwrite in
     * @param toInsert : The string which is put in the into string.
     * @param startPos : Start position where it should be inserted at.
     * @return The resulting string.
     */
    string overwrite(string into, string toInsert, unsigned long startPos) {
        unsigned long len = into.length();
        if (len == 0) {
            return toInsert;
        }
        unsigned long lenInsert = toInsert.length();
        if (lenInsert == 0) {
            return into;
        }
        ostringstream ss;
        // no fault check from here!
        ss << into.substr(0, startPos - 1);
        ss << toInsert;
        if (startPos - 1 + lenInsert <= len - 1) {
            ss << into.substr(ss.str().length(), into.length());
        }
        return ss.str();
    }

    string make_top_sections() {
        ostringstream ss;
        const int s = 5;
        for (int i = 0; i < 6; ++i) {
            ss << '|' << (i + 1) << space(s);
        }
        ss << '|';
        return ss.str();
    }

    string make_regular_sections(string c, int *val) {
        ostringstream ss;
        const int s = 6;
        ss << '|';
        if (*val > 0) {
            ss << c << ':' << *val;
        } else {
            ss << '|' << space(s);
        }
        ss << '|';
        return ss.str();
    }
};





#endif
