#include <chrono>
#include <memory>
#include <functional>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"

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

    //service clients for lifecycle management
    localization_client_ = this->create_client<nav2_msgs::srv::ManageLifecycleNodes>(
      "/lifecycle_manager_localization/manage_nodes");
    navigation_client_ = this->create_client<nav2_msgs::srv::ManageLifecycleNodes>(
      "/lifecycle_manager_navigation/manage_nodes");

    //wait services and activate Nav2 nodes
    startup_nav2();
  }

private:
  void startup_nav2()
  {
    RCLCPP_INFO(this->get_logger(), "Waiting for lifecycle management services...");
    
    //wait for localization service
    while (!localization_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for localization service");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for /lifecycle_manager_localization/manage_nodes service...");
    }

    //wait for navigation service
    while (!navigation_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for navigation service");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for /lifecycle_manager_navigation/manage_nodes service...");
    }

    RCLCPP_INFO(this->get_logger(), "Lifecycle management services available");

    //startup localization nodes
    auto localization_request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
    localization_request->command = nav2_msgs::srv::ManageLifecycleNodes::Request::STARTUP;
    
    auto localization_future = localization_client_->async_send_request(localization_request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), localization_future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      auto result = localization_future.get();
      if (result->success) {
        RCLCPP_INFO(this->get_logger(), "Localization nodes started successfully");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to start localization nodes");
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call localization service");
    }

    //startup navigation nodes
    auto navigation_request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
    navigation_request->command = nav2_msgs::srv::ManageLifecycleNodes::Request::STARTUP;
    
    auto navigation_future = navigation_client_->async_send_request(navigation_request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), navigation_future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      auto result = navigation_future.get();
      if (result->success) {
        RCLCPP_INFO(this->get_logger(), "Navigation nodes started successfully");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to start navigation nodes");
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call navigation service");
    }

    // Give Nav2 nodes time to fully activate
    RCLCPP_INFO(this->get_logger(), "Waiting for Nav2 nodes to be ready...");
    rclcpp::sleep_for(std::chrono::seconds(3));
    RCLCPP_INFO(this->get_logger(), "Nav2 nodes ready");

    // Now publish initial pose after AMCL is ready
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

  void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (navigating_) {
      RCLCPP_WARN(this->get_logger(), "Already navigating: ignoring new request");
      return;
    }

    if (!action_client_->wait_for_action_server(1s)) {
      RCLCPP_WARN(this->get_logger(), "Navigate action server not available, ignoring goal");
      return;
    }
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = *msg;
    RCLCPP_INFO(this->get_logger(), "Sending navigation goal in frame '%s'",
                msg->header.frame_id.c_str());

    navigating_ = true;

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    send_goal_options.goal_response_callback = [this](rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr goal_handle) {
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        navigating_ = false;
      } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted, navigating...");
      }
    };

    send_goal_options.feedback_callback = [this](
      rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr,
      const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
      if (feedback) {
        RCLCPP_INFO(this->get_logger(), "Distance remaining: %.2f m", feedback->distance_remaining);
      }
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
    };

    action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arrived_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
  bool navigating_ = false;

  rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr localization_client_;
  rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr navigation_client_;

  double initial_x_;
  double initial_y_;
  double initial_yaw_;
  std::string initial_frame_id_;
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

