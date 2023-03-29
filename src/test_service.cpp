#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/detail/empty__struct.hpp"
#include "std_srvs/srv/detail/set_bool__struct.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "robot_patrol/srv/get_direction.hpp"

#include <chrono>
#include <cstdlib>
#include <future>
#include <memory>

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

    this->timer_->cancel();

    while (!client_->wait_for_service(0.5s)) {
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
    auto status = future.wait_for(0.5s);
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
        1s, std::bind(&ServiceClient::timer_callback, this), callback_group_);

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
    // RCLCPP_INFO_ONCE(this->get_logger(), "HOLAAA %f",this->laser_range.ranges[360] );
  }

  bool is_service_done() const { return this->service_done_; }
}; // ServiceClient

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto service_client = std::make_shared<ServiceClient>();
  while (!service_client->is_service_done()) {
    rclcpp::spin_some(service_client);
  }

  rclcpp::shutdown();
  return 0;
}