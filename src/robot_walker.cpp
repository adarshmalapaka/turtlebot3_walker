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

#include "../include/turtlebot3_walker/Tb3Walker.hpp"


Tb3Walker::Tb3Walker()
    : Node("walker"), state_(STOP) {
       auto pubTopicName = "cmd_vel";
       publisher_ = this->create_publisher<TWIST> (pubTopicName, 10);

       auto subTopicName = "/scan";
       auto subCallback = std::bind(&Tb3Walker::scan_callback, this, _1);
       subscription_ = this->create_subscription<SCAN>
                    (subTopicName, 10, subCallback);

       auto timerCallback = std::bind(&Tb3Walker::timer_callback, this);
       timer_ = this->create_wall_timer(100ms, timerCallback);
}

void Tb3Walker::scan_callback(const SCAN& msg) {
    scan_ = msg;
}

void Tb3Walker::timer_callback() {
    if (scan_.header.stamp.sec == 0)
      return;

    auto message = TWIST();

    switch (state_) {
        case FORWARD:
        if (detect_obstacle()) {
            state_ = STOP;
            message.linear.x = 0.0;
            message.linear.y = 0.0;
            message.linear.z = 0.0;
            publisher_->publish(message);
            RCLCPP_INFO_STREAM(this->get_logger(), "State = STOP");
        }
        break;

        case STOP:
        if (detect_obstacle()) {
            state_ = TURN;
            message.angular.z = 0.1;
            publisher_->publish(message);
            RCLCPP_INFO_STREAM(this->get_logger(), "State = TURN");
        } else {
            state_ = FORWARD;
            message.linear.x = 0.2;
            publisher_->publish(message);
            RCLCPP_INFO_STREAM(this->get_logger(), "State = FORWARD");
        }
        break;

        case TURN:
        if (!detect_obstacle()) {
            state_ = FORWARD;
            message.linear.x = 0.2;
            publisher_->publish(message);
            RCLCPP_INFO_STREAM(this->get_logger(), "State = FORWARD");
        }
        break;
    }
}

bool Tb3Walker::detect_obstacle() {
    auto distance = scan_.ranges[0];

    if (distance < 0.8) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Obstacle detected!");
        return true;
    }

    return false;
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Tb3Walker>());
  rclcpp::shutdown();
  return 0;
}
