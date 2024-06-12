#ifndef CLIENT_NODE_HPP_
#define CLIENT_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>
#include <cabot_msgs/msg/pose_log.hpp>
#include <cabot_msgs/msg/log.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <functional>
#include <thread>
#include <atomic>
#include <cmath>
#include <ctime>
#include <chrono>
#include <map>
#include <vector>
#include <string>
#include <sstream>
#include <unordered_map>
#include <memory>
#include <boost/algorithm/string.hpp>

#include "geoutil.hpp"
#include "cabot_rclcpp_util.hpp"

#include <InfluxDB.h>
#include <InfluxDBFactory.h>
#include <InfluxDBBuilder.h>
#include <Point.h>

//using namespace std::chrono_literals;

class Throttle{
  /*
  A decorator to throttle function calls. The wrapped function can only be called
  once every interval_seconds. Subsequent calls within the interval are ignored.

  :param interval_seconds: The minimum time interval between function calls.
  :return: The wrapper function.
  */
public:
  Throttle(double interval_seconds_)
    : interval_(std::chrono::duration<double>(interval_seconds_)), last_called_(std::chrono::steady_clock::now() - std::chrono::duration_cast<std::chrono::seconds>(interval_)){}
  // Use a mutable object to allow modification in nested scope
  template<typename F, typename... Args>
  void call(F&& f, Args&&... args){
    auto now = std::chrono::steady_clock::now();
    if(now - last_called_ >= interval_){
      last_called_ = now;
      std::invoke(std::forward<F>(f), std::forward<Args>(args)...);
    }
  }

private:
  std::chrono::duration<double> interval_;
  std::chrono::steady_clock::time_point last_called_;
};


class ClientNode : public rclcpp::Node{
public:
  ClientNode();

private:
  std::string robot_name_;
  //rclcpp::Parameter robot_name_param;
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
  std::unique_ptr<influxdb::InfluxDB> influxdb_;
  Anchor anchor_;
  double anchor_rotate_;
  Throttle throttle_;

  double pose_interval_;
  double cmd_vel_interval_;
  double odom_interval_;
  double diag_agg_interval_;
  double battery_interval_;
  double image_interval_;
  std::shared_ptr<Throttle> pose_log_throttle_;
  std::shared_ptr<Throttle> cmd_vel_throttle_;
  std::shared_ptr<Throttle> odom_throttle_;
  std::shared_ptr<Throttle> diag_agg_throttle_;
  std::shared_ptr<Throttle> battery_throttle_;
  std::shared_ptr<Throttle> image_throttle_;
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

  long long last_error_;

  // cv_bridge::CvBridge bridge_;
  //std::mutex image_mutex_;

  void send_point(influxdb::Point&& point);
  std::chrono::time_point<std::chrono::system_clock> get_nanosec(const rclcpp::Time& stamp);
  void euler_from_quaternion(const geometry_msgs::msg::Quaternion& q, double& roll, double& pitch, double& yaw);

  void pose_log_callback(const cabot_msgs::msg::PoseLog::SharedPtr msg);
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  std::vector<std::string> split(const std::string& str, char delimiter);
  void diagnostics_callback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);
  void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
  void battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg, const std::string& direction);
  std::string base64_encode(const unsigned char *data, size_t len);
  void activity_log_callback(const cabot_msgs::msg::Log::SharedPtr msg);

  int main();
};

#endif // CLIENT_NODE_HPP_
