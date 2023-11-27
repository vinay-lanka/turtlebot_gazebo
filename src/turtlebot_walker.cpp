/**
 * @file turtlebot_walker.cpp
 * @author your name (you@domain.com)
 * @brief Walker script for the turtlebot to implement roomba like functionality
 * @version 0.1
 * @date 2023-11-26
 *
 * @copyright Copyright (c) 2023 Vinay Lanka
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
 */

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

class turtlebot_walker : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new turtlebot walker object
   *
   */
  turtlebot_walker() : Node("turtlebot_walker") {
    laser_scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&turtlebot_walker::scan_callback, this, _1));
    velocity_pub =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

 private:
  /**
   * @brief laser scan callback on the /scan topic
   *
   * @param scan_msg Laser Scan Message
   */
  void scan_callback(const sensor_msgs::msg::LaserScan& scan_msg) {
    for (int i = 0; i < 30; i++) {
      if (scan_msg.ranges[i % 360] < 0.8) {
        move(0.0, -0.2);
      } else {
        move(0.2, 0.0);
      }
    }
  }

  /**
   * @brief move member function that publishes to the cmd_vel topic
   *
   * @param linear_vel
   * @param angular_vel
   */
  void move(float linear_vel, float angular_vel) {
    auto velocity_msg = geometry_msgs::msg::Twist();
    velocity_msg.linear.x = linear_vel;
    velocity_msg.angular.z = angular_vel;
    velocity_pub->publish(velocity_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub;
  rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<turtlebot_walker>());
  rclcpp::shutdown();
  return 0;
}
