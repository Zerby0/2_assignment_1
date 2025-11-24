#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>
#include <string>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;

struct Point2D {
    double x;
    double y;
};

struct Circle {
    double x;
    double y;
    double r;
};

class TableDetector : public rclcpp::Node
{
public:
    TableDetector()
    : Node("table_detector"), is_active_(false), tables_detected_(false)
    {
        // --- Parametri ---
        cluster_tolerance_ = 0.30;  // Distanza max tra punti per unirli (m)
        min_cluster_size_ = 5;      // Minimo punti per considerare un oggetto (filtra rumore)
        max_cluster_size_ = 90;     // Massimo punti: IMPORTANTE per filtrare i MURI
        
        // Raggio atteso del tavolo (filtro geometrico)
        min_table_radius_ = 0.15;   
        max_table_radius_ = 0.45;   

        // --- Subscribers ---
        activation_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/arrived", 10, std::bind(&TableDetector::activation_callback, this, _1));

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&TableDetector::scan_callback, this, _1));

        // --- Publishers ---
        table_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/detected_tables", 10);

        // --- TF ---
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "Table Detector (Circle Fit Version) initialized.");
    }

private:
    bool is_active_;
    bool tables_detected_;
    
    double cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;
    double min_table_radius_;
    double max_table_radius_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr activation_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr table_pub_;
    
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    void activation_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data && !is_active_) {
            is_active_ = true;
            tables_detected_ = false; // Reset per nuova detection
            RCLCPP_INFO(this->get_logger(), "Start signal received! Scanning for circular tables...");
        }
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (!is_active_ || tables_detected_) {
            return;
        }

        auto clusters = cluster_scan(msg);
        auto candidates = process_clusters(clusters);

        if (candidates.size() >= 3) {
            RCLCPP_INFO(this->get_logger(), "Success! Found %zu circular tables. Transforming...", candidates.size());
            publish_tables_in_odom(candidates);
            tables_detected_ = true; 
        } else {
            static rclcpp::Time last_log = this->now();
            if ((this->now() - last_log).seconds() > 2.0) {
                RCLCPP_INFO(this->get_logger(), "Searching... Found %zu candidates.", candidates.size());
                last_log = this->now();
            }
        }
    }

    // Raggruppa punti vicini (Euclidean Clustering)
    std::vector<std::vector<Point2D>> cluster_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        std::vector<std::vector<Point2D>> clusters;
        std::vector<Point2D> current_cluster;
        
        double angle_min = scan_msg->angle_min;
        double angle_inc = scan_msg->angle_increment;

        // 1. Converti in Cartesiano
        std::vector<Point2D> points;
        for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
            float r = scan_msg->ranges[i];
            if (r < scan_msg->range_min || r > scan_msg->range_max || std::isnan(r)) continue;

            double ang = angle_min + (i * angle_inc);
            points.push_back({r * std::cos(ang), r * std::sin(ang)});
        }

        if (points.empty()) return clusters;

        // 2. Clustering
        current_cluster.push_back(points[0]);
        for (size_t i = 1; i < points.size(); ++i) {
            double dx = points[i].x - points[i-1].x;
            double dy = points[i].y - points[i-1].y;
            double dist = std::sqrt(dx*dx + dy*dy);

            if (dist < cluster_tolerance_) {
                current_cluster.push_back(points[i]);
            } else {
                if (current_cluster.size() >= (size_t)min_cluster_size_) {
                    clusters.push_back(current_cluster);
                }
                current_cluster.clear();
                current_cluster.push_back(points[i]);
            }
        }
        if (current_cluster.size() >= (size_t)min_cluster_size_) {
            clusters.push_back(current_cluster);
        }
        return clusters;
    }

    // Analizza i cluster e applica il Circle Fitting
    std::vector<Point2D> process_clusters(const std::vector<std::vector<Point2D>>& clusters)
    {
        std::vector<Point2D> valid_centers;

        for (const auto& cluster : clusters) {
            // Filtro 1: I muri hanno troppi punti
            if (cluster.size() > (size_t)max_cluster_size_) continue;

            // Fit Geometrico (Least Squares)
            Circle c = fit_circle(cluster);

            // Filtro 2: Il raggio deve essere compatibile con un tavolo
            // Se è un muro dritto, il raggio tenderà a infinito
            if (c.r >= min_table_radius_ && c.r <= max_table_radius_) {
                valid_centers.push_back({c.x, c.y});
                RCLCPP_DEBUG(this->get_logger(), "Candidate: r=%.2f at (%.2f, %.2f)", c.r, c.x, c.y);
            }
        }
        return valid_centers;
    }

    // Algoritmo di Kasa (Algebraic Circle Fit) - Robust Least Squares
    // Risolve il sistema lineare per trovare (Center_X, Center_Y, Radius)
    Circle fit_circle(const std::vector<Point2D>& points) {
        double sum_x = 0, sum_y = 0, sum_x2 = 0, sum_y2 = 0;
        double sum_xy = 0, sum_x3 = 0, sum_y3 = 0, sum_xy2 = 0, sum_x2y = 0;
        size_t N = points.size();

        for (const auto& p : points) {
            double x = p.x;
            double y = p.y;
            double x2 = x*x;
            double y2 = y*y;
            
            sum_x += x;
            sum_y += y;
            sum_x2 += x2;
            sum_y2 += y2;
            sum_xy += x*y;
            sum_x3 += x2*x;
            sum_y3 += y2*y;
            sum_xy2 += x*y2;
            sum_x2y += x2*y;
        }

        double C = N * sum_x2 - sum_x * sum_x;
        double D = N * sum_xy - sum_x * sum_y;
        double E = N * sum_x3 + N * sum_xy2 - (sum_x2 + sum_y2) * sum_x;
        double G = N * sum_y2 - sum_y * sum_y;
        double H = N * sum_x2y + N * sum_y3 - (sum_x2 + sum_y2) * sum_y;

        double denominator = C * G - D * D;

        Circle c{0, 0, 0};
        
        // Se il denominatore è quasi zero, i punti sono allineati (Linea retta/Muro)
        if (std::abs(denominator) < 1e-5) {
            c.r = 999.0; // Raggio infinito -> Muro
            return c;
        }

        double a = (G * E - D * H) / denominator;
        double b = (C * H - D * E) / denominator;

        c.x = a / 2.0;
        c.y = b / 2.0;
        c.r = std::sqrt(c.x*c.x + c.y*c.y + (sum_x2 + sum_y2)/N - 2*(c.x*sum_x + c.y*sum_y)/N);

        return c;
    }

    void publish_tables_in_odom(const std::vector<Point2D>& local_centers)
    {
        geometry_msgs::msg::PoseArray pose_array_msg;
        pose_array_msg.header.frame_id = "odom";
        pose_array_msg.header.stamp = this->now();

        try {
            geometry_msgs::msg::TransformStamped t = 
                tf_buffer_->lookupTransform("odom", "base_scan", tf2::TimePointZero);

            size_t limit = std::min(local_centers.size(), (size_t)3);
            
            for (size_t i = 0; i < limit; ++i) {
                geometry_msgs::msg::PointStamped p_scan, p_odom;
                p_scan.header.frame_id = "base_scan";
                p_scan.point.x = local_centers[i].x;
                p_scan.point.y = local_centers[i].y;
                p_scan.point.z = 0.0;

                tf2::doTransform(p_scan, p_odom, t);

                geometry_msgs::msg::Pose pose;
                pose.position = p_odom.point;
                pose.orientation.w = 1.0;
                pose_array_msg.poses.push_back(pose);
            }
            table_pub_->publish(pose_array_msg);

        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "TF Error: %s", ex.what());
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TableDetector>());
    rclcpp::shutdown();
    return 0;
}