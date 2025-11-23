#include "two_assignament_1/driver_node.hpp"

#include <chrono>
#include <memory>
// Single-file implementation: class definition + implementation in this cpp

#include <chrono>
#include <memory>
#include <functional>
#include <string>
#include <cmath>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

// Hardcoded configuration constants (edit here)
static constexpr char INPUT_TOPIC[] = "target";
static constexpr char ARRIVED_TOPIC[] = "arrived";
static constexpr double INITIAL_X = 0.0;
static constexpr double INITIAL_Y = 0.0;
static constexpr double INITIAL_YAW = 0.0; // radians
static constexpr char INITIAL_FRAME_ID[] = "map";

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;

namespace two_assignament_1
{

class DriverNode : public rclcpp::Node
{
public:
  explicit DriverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arrived_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;
  geometry_msgs::msg::PoseStamped goal_pose_;

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_;
  bool navigating_ = false;

  // hardcoded initial pose parameters (always used)
  double initial_x_ = 0.0;
  double initial_y_ = 0.0;
  double initial_yaw_ = 0.0;
  std::string initial_frame_id_ = "map";
  // initial straight-line distance from start to current goal (used for feedback percent)
  double initial_distance_ = 0.0;
};

DriverNode::DriverNode(const rclcpp::NodeOptions & options)
: Node("driver_node", options)
{
  // Hardcoded settings (from top-of-file constexpr)
  std::string input_topic = INPUT_TOPIC;     // topic for goals
  std::string arrived_topic = ARRIVED_TOPIC; // topic where the node publishes arrival

  initial_x_ = INITIAL_X;    // initial X position 
  initial_y_ = INITIAL_Y;    // initial Y position 
  initial_yaw_ = INITIAL_YAW;  // initial yaw (radians)
  initial_frame_id_ = INITIAL_FRAME_ID; // id initial pose

  subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    input_topic, 10,
    std::bind(&DriverNode::target_callback, this, std::placeholders::_1));

  arrived_pub_ = this->create_publisher<std_msgs::msg::Bool>(arrived_topic, 10);
  initialpose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1);
  action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
 
  {
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
}

void DriverNode::target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (navigating_) {
    RCLCPP_INFO(this->get_logger(), "Already navigating to a goal — ignoring new request");
    return;
  }

  if (!action_client_->wait_for_action_server(5s)) {
    RCLCPP_WARN(this->get_logger(), "Navigate action server not available");
    return;
  }

  goal_pose_ = *msg;

  // compute straight-line initial distance from hardcoded initial pose to goal (meters)
  {
    double dx = goal_pose_.pose.position.x - initial_x_;
    double dy = goal_pose_.pose.position.y - initial_y_;
    initial_distance_ = std::sqrt(dx * dx + dy * dy);
    if (initial_distance_ <= 0.0) {
      initial_distance_ = 0.0;
    }
  }

  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose = goal_pose_;

  navigating_ = true;

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  send_goal_options.goal_response_callback = [this](std::shared_future<rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr> future) {
    goal_handle_ = future.get();
    if (!goal_handle_) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      navigating_ = false;
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, navigating...");
    }
  };

  send_goal_options.feedback_callback = [this](
    rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
    if (!feedback) return;
    // nav2 NavigateToPose feedback typically provides `distance_remaining` (meters)
    double remaining = 0.0;
    // guard: try to read distance_remaining if present
    remaining = feedback->distance_remaining;
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

  action_client_->async_send_goal(goal_msg, send_goal_options);
}

}  // namespace two_assignament_1

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<two_assignament_1::DriverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
