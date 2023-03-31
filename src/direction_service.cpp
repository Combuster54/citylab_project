#include <chrono>
#include <functional>
#include <future>
#include <inttypes.h>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include "robot_patrol/srv/get_direction.hpp"

#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <unistd.h>

using GetDirectionMsg = robot_patrol::srv::GetDirection;

class DirectionService : public rclcpp::Node {

public:
  DirectionService() : Node("direction_service_node") {

    this->laser_range = {0};

    // Callback Group
    callback_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    srv_ = this->create_service<GetDirectionMsg>(
        "/direction_service",
        std::bind(&DirectionService::service_callback, this,
                  std::placeholders::_1, std::placeholders::_2),
        ::rmw_qos_profile_default, callback_group_);

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "Patroll Server is READY!");
  }

private:
  rclcpp::Service<GetDirectionMsg>::SharedPtr srv_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::CallbackGroup::SharedPtr laser_callback_G1;

  float min_left_laser;
  float min_right_laser;
  float min_front_laser;
  sensor_msgs::msg::LaserScan laser;
  std::vector<float> laser_range;

  void
  service_callback(const std::shared_ptr<GetDirectionMsg::Request> request,
                   const std::shared_ptr<GetDirectionMsg::Response> response) {

    RCLCPP_INFO(this->get_logger(), "PatrolService has been called!");
    laser = request->laser_data;
    laser_range = laser.ranges;

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

    // DEBUG
    //  RCLCPP_INFO(this->get_logger(), "Front: %f - Left: %f - Right %f ",
    //              this->min_front_laser,this->min_left_laser,
    //              this->min_right_laser);

    // FrontLaser
    if (this->min_front_laser > 0.4 &&
        (this->min_right_laser > 0.2 && this->min_right_laser < 0.3)) {
      response->direction = "Move forward";
    }
    // Left
    else if (this->min_right_laser <= 0.2 || this->min_front_laser < 0.4) {
      response->direction = "Turn left";
    }
    // Right
    else if (this->min_right_laser > 0.3 ) {
      response->direction = "Turn right";
    }
  }
};

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  auto directionServer = std::make_shared<DirectionService>();
  rclcpp::spin(directionServer);

  rclcpp::shutdown();
  return 0;
}
