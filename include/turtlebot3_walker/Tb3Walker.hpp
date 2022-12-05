// Copyright 2022 Adarsh M
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

using SCAN = sensor_msgs::msg::LaserScan;
using TWIST = geometry_msgs::msg::Twist;

typedef enum {
  FORWARD = 0,
  STOP,
  TURN,
} StateType;

class Tb3Walker : public rclcpp::Node {
 public:
    Tb3Walker();

 private:
    void timer_callback();
    void scan_callback(const SCAN&);
    bool detect_obstacle();
    rclcpp::Publisher<TWIST>::SharedPtr publisher_;
    rclcpp::Subscription<SCAN>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    SCAN scan_;
    float left_dist_;
    float center_dist_;
    float right_dist_;
    StateType state_;
};
