#include <string>
#include <opencv2/core.hpp> 
#include <opencv2/core/types.hpp>
#include <opencv2/calib3d.hpp> 
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class ApriltagManagerNode : public rclcpp::Node {
  public:
    ApriltagManagerNode(): Node("apriltag_manager"){
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      cameraInfoSub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/rgb_camera/camera_info", 3,
        std::bind(&ApriltagManagerNode::on_camera_info_received, this, std::placeholders::_1)
      );   
      detectionsSub_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
        "apriltag/detections", 3,
        std::bind(&ApriltagManagerNode::on_detection_received, this, std::placeholders::_1)
      );
      target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/target", 3);     
    }

  private:
    void on_camera_info_received(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
      if (camera_info_fetched_) return;     
      
      camera_matrix_ = cv::Mat::zeros(3, 3, CV_64F);
      for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j)
        camera_matrix_.at<double>(i, j) = static_cast<double>(msg->k[i*3 + j]); 

      dist_coeffs_ = cv::Mat::zeros((int)msg->d.size(), 1, CV_64F);
      for (long unsigned int i = 0; i < msg->d.size(); ++i)
        dist_coeffs_.at<double>(i, 0) = static_cast<double>(msg->d[i]);

      RCLCPP_INFO(this->get_logger(), "CameraInfo received");
      camera_info_fetched_ = true;
      cameraInfoSub_.reset(); // To avoid unuseful reassignment of the same values
    }  

    void on_detection_received(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg) {
      if (! camera_info_fetched_) {
        RCLCPP_WARN(this->get_logger(), "Detections received unprocessable, no CameraInfo received yet");
        return;
      }
        
      cv::Vec3d avg_t(0,0,0); 
      double s = TAG_SIZE / 2;
      std::vector<cv::Point3d> obj_points{
        {-s,  s, 0.0},
        { s,  s, 0.0},
        { s, -s, 0.0},
        {-s, -s, 0.0}
      };  
      cv::Mat rvec, tvec;    

      RCLCPP_DEBUG(this->get_logger(), "detections.size() = %zu", msg->detections.size());
      for (const auto &detection : msg->detections) {
        RCLCPP_DEBUG(this->get_logger(), "Processing tag id=%d", detection.id);

        std::vector<cv::Point2d> img_points{
          {detection.corners[0].x, detection.corners[0].y},
          {detection.corners[1].x, detection.corners[1].y},
          {detection.corners[2].x, detection.corners[2].y},
          {detection.corners[3].x, detection.corners[3].y}
        };

        if (!cv::solvePnP(obj_points, img_points, camera_matrix_, dist_coeffs_, rvec, tvec,
          false, cv::SOLVEPNP_IPPE_SQUARE)) {
            RCLCPP_WARN(this->get_logger(), "solvePnP failed for tag %d", detection.id);
            return;
        }

        if (tvec.empty() || tvec.total() < 3) {
          RCLCPP_WARN(this->get_logger(), "tvec empty or unexpected size for tag %d", detection.id);
          continue;
        }
        avg_t[0] += tvec.at<double>(0,0);
        avg_t[1] += tvec.at<double>(1,0);
        avg_t[2] += tvec.at<double>(2,0);
      }
      if (msg->detections.empty()) {
        RCLCPP_WARN(this->get_logger(), "No detections to process");
        return;
      }
      avg_t[0] /= msg->detections.size();
      avg_t[1] /= msg->detections.size();
      avg_t[2] /= msg->detections.size();
      
      geometry_msgs::msg::PoseStamped target_ref_camera;
      target_ref_camera.header.frame_id = CAMERA_FRAME;
      target_ref_camera.header.stamp = this->now();
      target_ref_camera.pose.position.x = avg_t[0];
      target_ref_camera.pose.position.y = avg_t[1];
      target_ref_camera.pose.position.z = avg_t[2];
      target_ref_camera.pose.orientation.x = 0.0;
      target_ref_camera.pose.orientation.y = 0.0;
      target_ref_camera.pose.orientation.z = 0.0;
      target_ref_camera.pose.orientation.w = 1.0;

      geometry_msgs::msg::PoseStamped target_ref_map;
      try {
        geometry_msgs::msg::TransformStamped tf_camera_map = 
          tf_buffer_->lookupTransform(MAP_FRAME, CAMERA_FRAME, tf2::TimePointZero);        
        tf2::doTransform(target_ref_camera, target_ref_map, tf_camera_map);
      }
      catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform target from camera to map: %s", ex.what());
        return;
      }

      target_pub_->publish(target_ref_map);
      RCLCPP_INFO(this->get_logger(), "Published target pose from map: x=%.3f y=%.3f z=%.3f", 
        target_ref_map.pose.position.x, target_ref_map.pose.position.y, target_ref_map.pose.position.z);
    }

    const std::string MAP_FRAME = "map";
    const std::string CAMERA_FRAME = "external_camera/link/rgb_camera";
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    bool camera_info_fetched_ = false;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cameraInfoSub_;
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detectionsSub_;

    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    const double TAG_SIZE = 0.05;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
};

int main(int argc, char ** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ApriltagManagerNode>());
  rclcpp::shutdown();
  return 0;
}
