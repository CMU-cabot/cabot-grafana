#ifndef CLIENT_NODE_HPP_
#define CLIENT_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>
#include <cabot_msgs/msg/pose_log.hpp>
#include <cabot_msgs/msg/log.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <string>
#include <memory>

#include "geoutil.hpp"
#include "cabot_rclcpp_util.hpp"

#include <InfluxDB.h>
#include <InfluxDBFactory.h>

using namespace std::chrono_literals;

class ClientNode : public rclcpp::Node{
public:
  ClientNode();

private:
  std::string robot_name_;
  rclcpp::Parameter robot_name_param;
  std::vector<std::string> robot_names_;
  std::string host_;
  std::string token_;
  std::string org_;
  std::string bucket_;
  std::string anchor_file_;
  std::string battery_topic_;
  std::string image_left_topic_;
  std::string image_center_topic_;
  std::string image_right_topic_;

  // std::shared_ptr<influxdb::InfluxDBClient> client_;
  // std::shared_ptr<influxdb::WriteApi> write_api_;
  rclcpp::Logger logger_;
  std::atomic<double> last_error_{0.0};

  rclcpp::Subscription<cabot_msgs::msg::PoseLog>::SharedPtr pose_log_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_agg_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_left_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_center_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_right_sub_;
  rclcpp::Subscription<cabot_msgs::msg::Log>::SharedPtr event_sub_;

  // cv_bridge::CvBridge bridge_;
  std::mutex image_mutex_;

  void send_point(influxdb::Point& point);
  rclcpp::Time get_nanosec(const rclcpp::Time& stamp = rclcpp::Time()) const;
  void pose_log_callback(const cabot_msgs::msg::PoseLog::SharedPtr msg, double interval);
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg, double interval);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg, double interval);
  void diagnostics_callback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg, double interval);
  void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
  void battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg, double interval);
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg, double interval, const std::string& direction);
  void activity_log_callback(const cabot_msgs::msg::Log::SharedPtr msg);
};

#endif // CLIENT_NODE_HPP_
