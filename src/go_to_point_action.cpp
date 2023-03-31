#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_patrol/action/go_to_point.hpp"

#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <unistd.h>

using std::placeholders::_1;
using std::placeholders::_2;

using namespace std::chrono_literals;

class GoToPoint : public rclcpp::Node {

public:
  int i = 0;
  using Point = robot_patrol::action::GoToPoint;
  using GoalHandlePoint = rclcpp_action::ServerGoalHandle<Point>;
  explicit GoToPoint(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("gotopoint_node", options) {

    using namespace std::placeholders;

    // typename Server<ActionT>::SharedPtr create_server(
    //     rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
    //     node_base_interface,
    //     rclcpp::node_interfaces::NodeClockInterface::SharedPtr
    //     node_clock_interface,
    //     rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr
    //         node_logging_interface,
    //     rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr
    //         node_waitables_interface,
    //     const std::string &name, typename Server<ActionT>::GoalCallback
    //     handle_goal, typename Server<ActionT>::CancelCallback handle_cancel,
    //     typename Server<ActionT>::AcceptedCallback handle_accepted,
    //     const rcl_action_server_options_t &options =
    //         rcl_action_server_get_default_options(),
    //     rclcpp::CallbackGroup::SharedPtr group = nullptr)

    // Callback Group
    callback_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    this->action_server_ = rclcpp_action::create_server<Point>(
        this, "go_to_point", std::bind(&GoToPoint::handle_goal, this, _1, _2),
        std::bind(&GoToPoint::handle_cancel, this, _1),
        std::bind(&GoToPoint::handle_accepted, this, _1),
        rcl_action_server_get_default_options(), callback_group_);

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "GotoPointAction Server is READY!");

    // LaserScan G1
    laser_callback_G1 =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    rclcpp::SubscriptionOptions options1;
    options1.callback_group = laser_callback_G1;

    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&GoToPoint::odom_callback, this, _1), options1);

    // subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    //     "odom", 10, std::bind(&GoToPoint::odom_callback, this, _1));

    timer_ = this->create_wall_timer(
        250ms, std::bind(&GoToPoint::timer_callback, this), callback_group_);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::CallbackGroup::SharedPtr laser_callback_G1;
  // Action Server
  rclcpp_action::Server<Point>::SharedPtr action_server_;
  std::shared_ptr<GoalHandlePoint> goal_handle_timer;
  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  geometry_msgs::msg::Point32 last_goal_position;
  // Odom_variables
  float x = 0.0;
  float y = 0.0;
  float theta = 0.0;
  bool timer_on = false;
  bool first_goal = true;
  void timer_callback() {

    if (timer_on) {

      auto feedback = std::make_shared<Point::Feedback>();
      feedback->current_pos.x = this->x;
      feedback->current_pos.y = this->y;
      feedback->current_pos.z = this->theta;

      goal_handle_timer->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(),
                  "[Feedback] -> [X,Y,Theta] = [ %f, %f, %f ]",
                  feedback->current_pos.x, feedback->current_pos.y,
                  feedback->current_pos.z);
    }
  }
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const Point::Goal> goal) {

    RCLCPP_INFO(this->get_logger(),
                "[Action_Server] Goal recieved, time to move!");

    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandlePoint> goal_handle) {

    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandlePoint> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a
    // new thread
    std::thread{std::bind(&GoToPoint::execute, this, _1), goal_handle}.detach();

    this->goal_handle_timer = goal_handle;
  }

  void execute(const std::shared_ptr<GoalHandlePoint> goal_handle) {

    RCLCPP_INFO(this->get_logger(), "Executing goal");

    const auto goal = goal_handle->get_goal();

    geometry_msgs::msg::Point32 goal_position;

    goal_position = goal->goal_pos;

    // auto x_goal = goal_position.x;
    // auto y_goal = goal_position.y;
    // auto theta_goal = (goal_position.z * 3.1416) / 180;
    goal_position.z = (goal_position.z * 3.1416) / 180;

    RCLCPP_INFO(this->get_logger(), "theta_goal = %f", goal_position.z);

    //  dx = 0.600417
    //  dy = -0.001389
    float dx;
    float dy;
    double euclidian_dist;

    if (first_goal) {

      RCLCPP_INFO(this->get_logger(), "First goal");

      dx = goal_position.x - this->x;
      dy = goal_position.y - this->y;

      if (abs(dx) < 0.0099) {
        dx = 0.0;
      }
      if (abs(dy) < 0.0099) {
        dy = 0.0;
      }

    } else {

      RCLCPP_INFO(this->get_logger(), "Another goal");

      dx = goal_position.x - last_goal_position.x;
      dy = goal_position.y - last_goal_position.y;
      if (abs(dx) < 0.0099) {
        dx = 0.0;
      }
      if (abs(dy) < 0.0099) {
        dy = 0.0;
      }
    }

    euclidian_dist = sqrt(pow(goal_position.x - this->x, 2) +
                          pow(goal_position.y - this->y, 2));
    first_goal = false;

    RCLCPP_INFO(this->get_logger(), "dx = %f", dx);
    RCLCPP_INFO(this->get_logger(), "dy = %f", dy);

    int T = (float(euclidian_dist / 0.05) * 1000 * 1000);
    T = T - (T % 10000);

    RCLCPP_INFO(this->get_logger(), "T = %i", T);

    float angle = atan(dy / dx);

    RCLCPP_INFO(this->get_logger(), "angle = %f", angle);
    auto result = std::make_shared<Point::Result>();
    auto move = geometry_msgs::msg::Twist();
    // Linear: 0.19 m/s max, Angular: 0.49 rad/s max

    bool rotate = false;
    bool forward_goal = false;
    bool goal_rotate = false;
    this->timer_on = true;
    while (true) {

      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {

        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      // First Flag
      if (!rotate) {
        // angle = -0.003524
        // No rotations
        if (angle == 0) {
          rotate = true;
          RCLCPP_INFO(this->get_logger(), "First flag complete");
        } // RIGHT
        else if (angle < 0) {
          // angle = -1.525513
          // theta = -1.522463
          RCLCPP_INFO(this->get_logger(), "this->theta =  %f ", this->theta);
          if (!(this->theta > angle - 0.035 && this->theta < angle + 0.035)) {
            move.linear.x = 0.0;
            move.angular.z = -0.05;
            publisher_->publish(move);
            usleep(500000);
          } else {
            move.linear.x = 0.0;
            move.angular.z = 0.0;
            publisher_->publish(move); // stop
            rotate = true;
            RCLCPP_INFO(this->get_logger(), "First flag complete");
          }

        } // LEFT
        else {

          RCLCPP_INFO(this->get_logger(), "this->theta =  %f ", this->theta);
          if (!(this->theta > angle - 0.035 && this->theta < angle + 0.035)) {
            move.linear.x = 0.0;
            move.angular.z = 0.05;
            publisher_->publish(move);
            usleep(500000);
            move.linear.x = 0.0;
            move.angular.z = 0.0;
            publisher_->publish(move);
          } else {
            rotate = true;
            RCLCPP_INFO(this->get_logger(), "First flag complete");
          }
        }
      }
      // Second Flag Real Robot
      if (!(forward_goal) && rotate) {

        while (T > 0) {
          move.linear.x = 0.05;
          move.angular.z = 0.0;
          publisher_->publish(move);
          usleep(1000);
          T -= 1000;
        }
        move.linear.x = 0.0;
        move.angular.z = 0.0;
        publisher_->publish(move);
        forward_goal = true;
        RCLCPP_INFO(this->get_logger(), "Second flag complete");
      }

      // Third Flag
      if (!(goal_rotate) && rotate && forward_goal) {

        if (goal_position.z == 0) {
          goal_rotate = true;
          RCLCPP_INFO(this->get_logger(), "Last flag complete");
        } // RIGHT
        else if (goal_position.z < 0) {
          RCLCPP_INFO_ONCE(this->get_logger(), "Last flag: RIGHT");

          if (!(this->theta < goal_position.z - 0.035 &&
                this->theta > goal_position.z + 0.035)) {
            move.linear.x = 0.0;
            move.angular.z = -0.1;
            publisher_->publish(move);
            usleep(500000); // sleep
            move.linear.x = 0.0;
            move.angular.z = 0.0;
            publisher_->publish(move); // stop
          } else {
            goal_rotate = true;
            RCLCPP_INFO(this->get_logger(), "Last flag complete");
          }
        } // LEFT
        else {
          RCLCPP_INFO_ONCE(this->get_logger(), "Last flag: LEFT");

          if (!(this->theta > goal_position.z - 0.035 &&
                this->theta < goal_position.z + 0.035)) {
            move.linear.x = 0.0;
            move.angular.z = 0.1;
            publisher_->publish(move);
            usleep(500000);
            move.linear.x = 0.0;
            move.angular.z = 0.0;
            publisher_->publish(move);
          } else {
            goal_rotate = true;
            RCLCPP_INFO(this->get_logger(), "Last flag complete");
          }
        }
      }

      if (goal_rotate && rotate && forward_goal) {
        break;
      }
    }

    this->timer_on = false;
    last_goal_position = goal->goal_pos;
    move.linear.x = 0.0;
    move.angular.z = 0.0;
    publisher_->publish(move);
    result->status = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "[STATUS GOAL] = %s", result->status ? "TRUE" : "FALSE");
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {

    RCLCPP_INFO_ONCE(this->get_logger(), "Go odometry!");
    tf2::Quaternion quat_tf;
    geometry_msgs::msg::Quaternion quat_msg = msg->pose.pose.orientation;
    tf2::fromMsg(quat_msg, quat_tf);
    double roll{}, p{}, yaw{};
    tf2::Matrix3x3 m(quat_tf);
    m.getRPY(roll, p, yaw);

    this->x = msg->pose.pose.position.x;
    this->y = msg->pose.pose.position.y;
    this->theta = yaw;

    RCLCPP_INFO_ONCE(this->get_logger(), "  x =   %f", this->x);
    RCLCPP_INFO_ONCE(this->get_logger(), "  y =   %f", this->y);
    RCLCPP_INFO_ONCE(this->get_logger(), "theta = %f", this->theta);
  }
}; // class GoToPointAction

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  auto goToPointActionServer = std::make_shared<GoToPoint>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(goToPointActionServer);
  executor.spin();
  rclcpp::shutdown();

  return 0;
}