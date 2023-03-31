#include <chrono>
#include <functional>
#include <future>
#include <inttypes.h>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <unistd.h>

using namespace std::chrono_literals;

using std::placeholders::_1;

class Patrol : public rclcpp::Node {

public:
  Patrol() : Node("patrolling_node") {

    // Subscriber
    sub_laser = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&Patrol::scan_callback, this, _1));
    timer_move =
        this->create_wall_timer(500ms, std::bind(&Patrol::time_to_move, this));
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

  void stop() {
    this->velocity.linear.x = 0.0;
    this->velocity.angular.z = 0.0;
    publisher_->publish(this->velocity);
  }

  geometry_msgs::msg::Twist velocity;

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser;
  rclcpp::TimerBase::SharedPtr timer_move;
  float min_left_laser;
  float min_right_laser;
  float min_front_laser;
  std::vector<float> laser_range;

  void time_to_move() {

    filterScan(); // change values of min_front_laser, etc...
    // FRONT
    if (this->min_front_laser > 0.4 &&
        (this->min_right_laser > 0.2 && this->min_right_laser < 0.3)) {
      this->velocity.linear.x = 0.05;
      this->velocity.angular.z = 0.0;
    }
    // LEFT
    else if (this->min_right_laser <= 0.2 || this->min_front_laser < 0.4) {

      this->velocity.linear.x = 0.04;
      this->velocity.angular.z = 0.35;
    }
    // RIGHT
    else if (this->min_right_laser > 0.3) {

      this->velocity.linear.x = 0.04;
      this->velocity.angular.z = -0.35;
    }
    publisher_->publish(this->velocity);

    if (!(rclcpp::ok())) {

      this->velocity.linear.x = 0.0;
      this->velocity.angular.z = 0.0;
      publisher_->publish(this->velocity);
      
      timer_move->cancel();
    }
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    this->laser_range = msg->ranges;
  }
  void filterScan() {

    // Segment the scan of 180 to 540
    // 420-540 ---> LEFT
    std::vector<float> left_laser;
    std::copy(this->laser_range.begin() + 420, this->laser_range.begin() + 540,
              std::back_inserter(left_laser));

    // 300-419 ---> FRONT
    std::vector<float> front_laser;
    std::copy(this->laser_range.begin() + 300, this->laser_range.begin() + 419,
              std::back_inserter(front_laser));

    // 180-299 ---> RIGHT
    std::vector<float> right_laser;
    std::copy(this->laser_range.begin() + 180, this->laser_range.begin() + 299,
              std::back_inserter(right_laser));

    // Find the minimun laser for each side
    this->min_front_laser =
        *min_element(front_laser.begin(), front_laser.end());
    this->min_right_laser =
        *min_element(right_laser.begin(), right_laser.end());
    this->min_left_laser = *min_element(left_laser.begin(), left_laser.end());
  }
}; // class Patrol

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  auto patrol = std::make_shared<Patrol>();

  rclcpp::executors::MultiThreadedExecutor executor;

  executor.add_node(patrol);

  executor.spin();
  rclcpp::shutdown();
  patrol->stop();
  return 0;
} // Main
