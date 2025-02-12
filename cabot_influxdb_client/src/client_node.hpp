/*******************************************************************************
 * Copyright (c) 2024  Carnegie Mellon University and Miraikan
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *******************************************************************************/

#ifndef CLIENT_NODE_HPP_
#define CLIENT_NODE_HPP_

#include <tf2_ros/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <atomic>
#include <cmath>
#include <ctime>
#include <chrono>
#include <csignal>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <sstream>
#include <thread>
#include <utility>
#include <unordered_map>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <cabot_msgs/msg/log.hpp>
#include <cabot_msgs/msg/pose_log.hpp>
#include <cabot_msgs/msg/anchor.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "InfluxDB.hpp"
#include "InfluxPoint.hpp"
#include "cabot_rclcpp_util.hpp"
#include "geoutil.hpp"

class Throttle
{
  /*
  A decorator to throttle function calls. The wrapped function can only be called
  once every interval_seconds. Subsequent calls within the interval are ignored.

  :param interval_seconds_: The minimum time interval between function calls.
  :return: The wrapper function.
  */

public:
  explicit Throttle(double interval_seconds_)
  : interval_(std::chrono::duration<double>(interval_seconds_)), last_called_(
      std::chrono::steady_clock::now() -
      std::chrono::duration_cast<std::chrono::seconds>(interval_))
  {
  }
  // Use a mutable object to allow modification in nested scope
  template<typename F, typename ... Args>
  void call(F && f, Args &&... args)
  {
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    if (now - last_called_ >= interval_) {
      last_called_ = now;
      std::invoke(std::forward<F>(f), std::forward<Args>(args)...);
    }
  }

private:
  std::chrono::duration<double> interval_;
  std::chrono::steady_clock::time_point last_called_;
};

class ClientNode : public rclcpp::Node
{
public:
  ClientNode();
  virtual ~ClientNode();

private:
  std::string robot_name_;
  std::string host_;
  std::string token_;
  std::string org_;
  std::string bucket_;
  std::string anchor_file_;
  std::string image_left_topic_;
  std::string image_center_topic_;
  std::string image_right_topic_;
  std::string rotate_image_;
  std::vector<std::string> rotate_images_;
  std::vector<std::string> split_string(const std::string &str, char delimiter);

  InfluxDB client_;
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
  std::shared_ptr<Throttle> image_left_throttle_;
  std::shared_ptr<Throttle> image_center_throttle_;
  std::shared_ptr<Throttle> image_right_throttle_;
  rclcpp::Subscription<cabot_msgs::msg::Anchor>::SharedPtr anchor_sub_;
  rclcpp::Subscription<cabot_msgs::msg::PoseLog>::SharedPtr pose_log_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_agg_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_left_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_center_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_right_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_left_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_center_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_right_sub_;

  rclcpp::Subscription<cabot_msgs::msg::Log>::SharedPtr event_sub_;
  int64_t last_error_;

  std::chrono::time_point<std::chrono::system_clock> get_nanosec(const rclcpp::Time & stamp);
  void euler_from_quaternion(
    const geometry_msgs::msg::Quaternion & q, double & roll,
    double & pitch, double & yaw);
  void pose_log_callback(const cabot_msgs::msg::PoseLog::SharedPtr msg);
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void diagnostics_callback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);
  void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
  void battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg, const std::string & direction);
  void compressed_image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg, const std::string & direction);
  cv::Mat resize_with_aspect_ratio(const cv::Mat& image, int target_size);
  cv::Mat rotate_image(const cv::Mat& image, const std::string& direction);
  std::string base64_encode(const std::vector<uchar>& data);
  void activity_log_callback(const cabot_msgs::msg::Log::SharedPtr msg);

  int main();
};

#endif  // CLIENT_NODE_HPP_
