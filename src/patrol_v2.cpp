#include <chrono>
#include <functional>
#include <future>
#include <inttypes.h>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include "robot_patrol/action/go_to_point.hpp"
#include "robot_patrol/srv/get_direction.hpp"

#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <unistd.h>

using namespace std::chrono_literals;

using GetDirectionMsg = robot_patrol::srv::GetDirection;

class ServiceClient : public rclcpp::Node {

private:
  rclcpp::Client<GetDirectionMsg>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::CallbackGroup::SharedPtr laser_callback_G1;

  bool service_done_ = false;

  void timer_callback() {

    while (!client_->wait_for_service(1s)) {
      if (rclcpp::ok()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Client interrupted while waiting for service. Terminating...");
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Service Unavailable. Waiting for Service...");
    }

    // Subscriber
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&ServiceClient::scan_callback, this, std::placeholders::_1));

    auto request = std::make_shared<GetDirectionMsg::Request>();

    request->laser_data = this->laser_range;

    service_done_ = false;

    auto result_future = client_->async_send_request(
        request, std::bind(&ServiceClient::response_callback, this,
                           std::placeholders::_1));
  }

  void response_callback(rclcpp::Client<GetDirectionMsg>::SharedFuture future) {
    auto status = future.wait_for(1s);

    if (status == std::future_status::ready) {

      this->var_direction = future.get()->direction;

      RCLCPP_INFO(this->get_logger(), "Service returned: %s",
                  future.get()->direction.c_str());

      service_done_ = true;

    } else {

      RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
  }

public:
  std::string var_direction;
  sensor_msgs::msg::LaserScan laser_range;

  ServiceClient() : Node("service_client") {

    // Callback Group
    callback_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    client_ = this->create_client<GetDirectionMsg>("direction_service");

    timer_ = this->create_wall_timer(
        0.2s, std::bind(&ServiceClient::timer_callback, this), callback_group_);

    // LaserScan G1
    laser_callback_G1 =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    rclcpp::SubscriptionOptions options1;
    options1.callback_group = laser_callback_G1;

    // Subscriber
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&ServiceClient::scan_callback, this, std::placeholders::_1));
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    this->laser_range.ranges = msg->ranges;
    // RCLCPP_INFO_ONCE(this->get_logger(), "HOLAAA
    // %f",this->laser_range.ranges[360] );
  }

  bool is_service_done() const { return this->service_done_; }
}; // ServiceClient

class Patrol : public rclcpp::Node {

public:
  std::shared_ptr<ServiceClient> serviceClientClass;
  geometry_msgs::msg::Twist velocity;

  Patrol() : Node("patrolling_node") {

    serviceClientClass = std::make_shared<ServiceClient>();
    timer_move =
        this->create_wall_timer(500ms, std::bind(&Patrol::time_to_move, this));
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser;
  rclcpp::TimerBase::SharedPtr timer_move;
  float min_left_laser;
  float min_right_laser;
  float min_front_laser;

  void time_to_move() {

    // FRONT
    if (serviceClientClass->var_direction == "Move forward") {
      this->velocity.linear.x = 0.05;
      this->velocity.angular.z = 0.0;
    }
    // RIGHT
    else if (serviceClientClass->var_direction == "Turn right") {
      this->velocity.linear.x = 0.04;
      this->velocity.angular.z = -0.35;
    }
    // LEFT
    else if (serviceClientClass->var_direction == "Turn left") {
      this->velocity.linear.x = 0.04;
      this->velocity.angular.z = 0.35;
    }
    publisher_->publish(this->velocity);

    if (!(rclcpp::ok())) {

      this->velocity.linear.x = 0.0;
      this->velocity.angular.z = 0.0;
      publisher_->publish(this->velocity);
      timer_move->cancel();
      RCLCPP_INFO(this->get_logger(), "Stop timer");

    }
  }

}; // class Patrol

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  auto patrol = std::make_shared<Patrol>();
  auto service_client = patrol->serviceClientClass;

  rclcpp::executors::MultiThreadedExecutor executor;

  executor.add_node(patrol);
  executor.add_node(service_client);

  executor.spin();
  rclcpp::shutdown();

  return 0;
} // Main