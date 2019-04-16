/*                Copyright (C) 2000-2019, Danilo Tardioli               *
 *           Centro Universitario de la Defensa Zaragoza, SPAIN          *
 *
 *  Contact Addresses: Danilo Tardioli                   dantard@unizar.es
 *
 *  This is free software; you can  redistribute it and/or  modify it
 *  under the terms of the GNU General Public License  as published by the
 *  Free Software Foundation;  either  version 2, or (at  your option) any
 *  later version.
 *
 *  This software is distributed  in the  hope  that  it will be   useful,
 *  but WITHOUT  ANY  WARRANTY;   without  even the implied   warranty  of
 *  MERCHANTABILITY  or  FITNESS FOR A  PARTICULAR PURPOSE.    See the GNU
 *  General Public License for more details.
 *
 *  You should have received  a  copy of  the  GNU General Public  License
 *  distributed with this code;  see file COPYING.   If not,  write to the
 *  Free Software  Foundation,  59 Temple Place  -  Suite 330,  Boston, MA
 *  02111-1307, USA.
 *
 *  As a  special exception, if you  link this  unit  with other  files to
 *  produce an   executable,   this unit  does  not  by  itself cause  the
 *  resulting executable to be covered by the  GNU General Public License.
 *  This exception does  not however invalidate  any other reasons why the
 *  executable file might be covered by the GNU Public License.
 *
 *----------------------------------------------------------------------*/

#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include "serial/serial.h"

#include <string>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/MultiEchoLaserScan.h>
#include <std_msgs/String.h>

#include <urad/urad.h>

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

int type = 0;
int mode = 3;    // mode
int f0 = 5;      // starting freq
int BW = 120;    // BW available (max 240 MHz)
int Ns = 200;    // n samples
int Ntar = 2;    // n targets of interest
int Rmax = 100;  // distance range
int MTI = 0;     // MTI
int Mth = 0;     // sesitive threshold (1-4)

std::string ON = "{ON}";
std::string OFF = "{OFF}";


void split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
}

size_t process(std::string data, std::vector<std::string> &list) {

    list.clear();

    size_t s_begin = data.find('{');
    size_t s_end = data.find('}');


    if (s_begin == std::string::npos || s_end == std::string::npos || s_end <= s_begin) {
        return 0;
    }

    data = data.substr(s_begin + 1, s_end - s_begin - 1);
    split(data, ';', list);


    return list.size();
}

void write(serial::Serial &ser, std::string s) {
    for (auto &e:s) {
        auto c = uint8_t(e);
        ser.write(&c, 1);
        usleep(1000);
    }
}

enum {
    RANGE, MLIDAR, LIDAR, ODOM, NATIVE, DIST, RAW, SPEED, ALL
};

struct RadarReading {
    RadarReading() = default;

    float dist, snr, speed;
    bool movement;
    bool operator < (const RadarReading& str) const
    {
        return (dist < str.dist);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "uRAD");

    ros::NodeHandle np;
    ros::NodeHandle nh("~");

    std::string port = "/dev/ttyACM0";
    nh.getParam("port", port);

    int baud = 230400;
    nh.getParam("baud", baud);

    int laser_points = 50;
    nh.getParam("laser_points", laser_points);

    nh.getParam("mode", mode);
    nh.getParam("ntar", Ntar);
    nh.getParam("mth", Mth);
    nh.getParam("rmax", Rmax);
    nh.getParam("f0", f0);
    nh.getParam("bw", BW);

    std::string output = "all";
    nh.getParam("output", output);

    if (output == "mlidar")
        type = MLIDAR;
    else if (output == "odom") {
        type = ODOM;
        Ntar = 1;
    } else if (output == "native")
        type = NATIVE;
    else if (output == "lidar")
        type = LIDAR;
    else if (output == "dist")
        type = DIST;
    else if (output == "speed")
        type = SPEED;
    else if (output == "raw")
        type = RAW;
    else if (output == "all")
        type = ALL;

    int debug = 0;
    nh.getParam("debug", debug);

    serial::Serial serial(port, uint32_t(baud), serial::Timeout::simpleTimeout(1000));

    try {

        if (!serial.isOpen()) {
            serial.open();
        }

    } catch (exception &e) {
        cerr << "Error opening port " << e.what() << endl;
    }

    std::string cfg = "{CFG;" +
                      std::to_string(mode) + ";" +
                      std::to_string(f0) + ";" +
                      std::to_string(BW) + ";" +
                      std::to_string(Ns) + ";" +
                      std::to_string(Ntar) + ";" +
                      std::to_string(Rmax) + ";" +
                      std::to_string(MTI) + ";" +
                      std::to_string(Mth) + "}";
    
    std::vector<std::string> list;

    while (ros::ok()) {
        std::string line = serial.readline();
        if (debug == 2) {
            std::cerr << line;
        }
        if (process(line, list)) {
            if (list.at(0) == "RAD") {

                int mode = atoi(list.at(1).c_str());
                int how_many = atoi(list.at(2).c_str());

                std::vector<RadarReading> readings;

                for (unsigned int i = 0; i < how_many; i++) {
                    if (6 + 4 * i < list.size()) {
                        RadarReading r;
                        r.dist = atof(list.at(3 + 4 * i).c_str())/1000.0;
                        r.speed = atof(list.at(4 + 4 * i).c_str())/1000.0;
                        r.snr = atof(list.at(5 + 4 * i).c_str())/1000.0;
                        r.movement = atoi(list.at(6 + 4 * i).c_str());
                        readings.push_back(r);
                    }
                }

                //std::sort(readings.begin(), readings.end());

                if (type == MLIDAR || type == ALL) {
                    static ros::Publisher pub = np.advertise<sensor_msgs::MultiEchoLaserScan>("urad/multiscan", 1000);
                    sensor_msgs::MultiEchoLaserScan m;
                    m.angle_max = 10 * M_PI / 180;
                    m.angle_min = -10 * M_PI / 180;
                    m.range_max = Rmax;
                    m.angle_increment = (m.angle_max - m.angle_min) / laser_points;
                    m.header.frame_id = "urad";
                    m.header.stamp = ros::Time::now();
                    for (int j = 0; j < laser_points; j++) {
                        sensor_msgs::LaserEcho range, intensity;
                        for (int i = 0; i < readings.size(); i++) {
                            range.echoes.push_back(readings.at(i).dist);
                            intensity.echoes.push_back(readings.at(i).snr);
                        }
                        m.ranges.push_back(range);
                        m.intensities.push_back(intensity);
                    }
                    pub.publish(m);
                }

                if (type == LIDAR || type == ALL) {
                    static ros::Publisher pub = np.advertise<sensor_msgs::LaserScan>("urad/scan", 1000);
                    sensor_msgs::LaserScan m;
                    m.angle_max = 10.5 * M_PI / 180;
                    m.angle_min = -10.5 * M_PI / 180;
                    m.range_max = Rmax;
                    m.angle_increment = (m.angle_max - m.angle_min) / laser_points;
                    m.header.frame_id = "urad";
                    m.header.stamp = ros::Time::now();
                    m.ranges.resize(laser_points);
                    m.intensities.resize(laser_points);
                    for (int i = 0; i < readings.size(); i++) {
                        for (int j = 0; j < laser_points; j++) {
                            if (i == j % 5) {
                                m.ranges.at(j) = readings.at(i).dist;
                                m.intensities.at(j) = readings.at(i).snr;
                            }
                        }
                    }
                    pub.publish(m);

                }

                if (type == ODOM || type == ALL) {
                    static ros::Publisher pub = np.advertise<nav_msgs::Odometry>("urad/odom", 1000);
                    nav_msgs::Odometry m;
                    m.header.frame_id = "urad";
                    m.header.stamp = ros::Time::now();
                    if (!readings.empty()) {
                        m.pose.pose.position.x = readings.at(0).dist;
                        m.twist.twist.linear.x = readings.at(0).speed;
                    }
                    pub.publish(m);
                }

                if (type == DIST || type == ALL) {
                    static ros::Publisher pub = np.advertise<std_msgs::Float64MultiArray>("urad/dist", 1000);
                    std_msgs::Float64MultiArray m;

                    std_msgs::MultiArrayDimension mad;
                    mad.size = readings.size();
                    mad.label = "Distance";
                    mad.stride = mad.size;

                    m.layout.dim.push_back(mad);
                    mad.label = "Speed";
                    m.layout.dim.push_back(mad);

                    for (int i = 0; i < readings.size(); i++) {
                        m.data.push_back(readings.at(i).dist);
                    }

                    for (int i = 0; i < readings.size(); i++) {
                        m.data.push_back(readings.at(i).speed);
                    }
                    pub.publish(m);
                }

                if (type == SPEED || type == ALL) {
                    static ros::Publisher pub = np.advertise<std_msgs::Float64MultiArray>("urad/speed", 1000);
                    std_msgs::Float64MultiArray m;

                    for (int i = 0; i < readings.size(); i++) {
                        m.data.push_back(readings.at(i).speed);
                    }
                    pub.publish(m);
                }

                if (type == NATIVE || type == ALL) {
                    static ros::Publisher pub = np.advertise<urad::urad>("urad/urad", 1000);
                    urad::urad m;
                    m.header.stamp = ros::Time::now();
                    m.header.frame_id = "urad";
                    for (int i = 0; i < readings.size(); i++) {
                        urad::target target;
                        target.pose.position.x = readings.at(i).dist;
                        target.twist.linear.x = readings.at(i).speed;
                        target.movement = readings.at(i).movement;
                        target.snr = readings.at(i).snr;
                        m.targets.push_back(target);
                    }
                    pub.publish(m);
                }

                if (type == RAW || type == ALL) {
                    static ros::Publisher pub = np.advertise<std_msgs::String>("urad/raw", 1000);
                    std_msgs::String m;
                    m.data = line;
                    pub.publish(m);
                }


            } else if (list.at(0) == "START") {
                write(serial, OFF);
                write(serial, cfg);
                write(serial, ON);
            } else if (list.at(0) == "DBG") {
                if (debug == 1) {
                    for (auto &e: list) {
                        std::cerr << e << ";";
                    }
                    std::cerr << std::endl;
                }
            }
        }
        ros::spinOnce();
    }
}

