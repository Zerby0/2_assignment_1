#include <chrono>
#include <memory>
#include <functional>
#include <string>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

static constexpr char INPUT_TOPIC[] = "target";
static constexpr char ARRIVED_TOPIC[] = "arrived";
static constexpr double INITIAL_X = 0.0;
static constexpr double INITIAL_Y = 0.0;
static constexpr double INITIAL_YAW = 0.0;
static constexpr char INITIAL_FRAME_ID[] = "map";

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;

namespace two_assignament_1
{

class DriverNode : public rclcpp::Node
{
public:
  explicit DriverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("driver_node"),
    initial_x_(INITIAL_X), initial_y_(INITIAL_Y), initial_yaw_(INITIAL_YAW), initial_frame_id_(INITIAL_FRAME_ID)
  {
    (void)options;
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      INPUT_TOPIC, 10, std::bind(&DriverNode::target_callback, this, std::placeholders::_1));

    arrived_pub_ = this->create_publisher<std_msgs::msg::Bool>(ARRIVED_TOPIC, 10);
    initialpose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1);

    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    geometry_msgs::msg::PoseWithCovarianceStamped init;
    init.header.stamp = this->now();
    init.header.frame_id = initial_frame_id_;
    init.pose.pose.position.x = initial_x_;
    init.pose.pose.position.y = initial_y_;
    init.pose.pose.position.z = 0.0;
    double cy = std::cos(initial_yaw_ * 0.5);
    double sy = std::sin(initial_yaw_ * 0.5);
    init.pose.pose.orientation.z = sy;
    init.pose.pose.orientation.w = cy;
    initialpose_pub_->publish(init);
    RCLCPP_INFO(this->get_logger(), "Published initial pose: x=%.2f y=%.2f yaw=%.2f", initial_x_, initial_y_, initial_yaw_);
  }

private:
  void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (navigating_) {
      RCLCPP_INFO(this->get_logger(), "Already navigating — ignoring new request");
      return;
    }

    if (!action_client_->wait_for_action_server()) {
      RCLCPP_WARN(this->get_logger(), "Navigate action server not available");
      return;
    }
    geometry_msgs::msg::PoseStamped incoming = *msg;
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = incoming;
    initial_distance_ = 0.0;
    RCLCPP_INFO(this->get_logger(), "Forwarding incoming pose in frame '%s' directly to Nav2",
                incoming.header.frame_id.c_str());

    navigating_ = true;

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    send_goal_options.goal_response_callback = [this](rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr goal_handle) {
      goal_handle_ = goal_handle;
      if (!goal_handle_) {
        // log rich info to help debugging why the goal was rejected
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        const auto & g = last_sent_goal_;
        RCLCPP_ERROR(this->get_logger(), "Rejected goal details: frame='%s' pos=(%.3f, %.3f, %.3f) quat=(%.3f, %.3f, %.3f, %.3f)",
                    g.header.frame_id.c_str(),
                    g.pose.position.x, g.pose.position.y, g.pose.position.z,
                    g.pose.orientation.x, g.pose.orientation.y, g.pose.orientation.z, g.pose.orientation.w);
        RCLCPP_ERROR(this->get_logger(), "Check Nav2 logs and frame/TF configuration (global_frame vs goal header)");
        navigating_ = false;
      } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, navigating...");
      }
    };

    send_goal_options.feedback_callback = [this](
      rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr,
      const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
      if (!feedback) return;
      double remaining = feedback->distance_remaining;
      double percent = 0.0;
      if (initial_distance_ > 0.0) {
        percent = 100.0 * (1.0 - (remaining / initial_distance_));
        percent = std::max(0.0, std::min(100.0, percent));
      }
      RCLCPP_INFO(this->get_logger(), "Navigation feedback: remaining=%.3f m, progress=%.1f%%", remaining, percent);
    };

    send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        std_msgs::msg::Bool msg;
        msg.data = true;
        arrived_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Navigation succeeded — published arrived=true");
      } else {
        RCLCPP_WARN(this->get_logger(), "Navigation failed or canceled — not publishing arrived=true");
      }
      navigating_ = false;
      goal_handle_.reset();
    };

    last_sent_goal_ = goal_msg.pose;
    action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arrived_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_;
  bool navigating_ = false;

  // store last sent goal for debugging when a goal is rejected
  geometry_msgs::msg::PoseStamped last_sent_goal_;


  double initial_x_;
  double initial_y_;
  double initial_yaw_;
  std::string initial_frame_id_;
  double initial_distance_ = 0.0;
};

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<two_assignament_1::DriverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

