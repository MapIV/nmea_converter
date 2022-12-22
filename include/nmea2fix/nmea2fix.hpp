// Copyright (c) 2019, Map IV, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of the Map IV, Inc. nor the names of its contributors
//   may be used to endorse or promote products derived from this software
//   without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef NMEA2FIX_H
#define NMEA2FIX_H

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nmea_msgs/msg/sentence.hpp>
#include <nmea_msgs/msg/gpgga.hpp>
#include <string>
#include <time.h>
#include <memory>
#include <cmath>
#include <chrono>

#include <rclcpp/rclcpp.hpp>


class Nmea2Fix:public rclcpp::Node
{
private:
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsatafix_pub_;
    rclcpp::Publisher<nmea_msgs::msg::Gpgga>::SharedPtr nmea_pub_;
    rclcpp::Subscription<nmea_msgs::msg::Sentence>::SharedPtr nmea_sub_;

    nmea_msgs::msg::Sentence sentence_;

    std::string sub_topic_name_,pub_fix_topic_name_,pub_gga_topic_name_;
    bool output_gga_;

    rclcpp::TimerBase::SharedPtr timer_;

    double stringToROSTime(std::string&, double);
    void nmea2fixConverter(const nmea_msgs::msg::Sentence,  sensor_msgs::msg::NavSatFix*, nmea_msgs::msg::Gpgga*);
    void nmeaCallback(const nmea_msgs::msg::Sentence::ConstSharedPtr msg);
    void timerCallback();
    double toSec(std_msgs::msg::Header &msg);


public:
    Nmea2Fix();
    ~Nmea2Fix();
};


#endif /*NMEA2FIX_H */
