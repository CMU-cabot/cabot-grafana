#ifndef POSE_LOG_CLIENT_HPP_
#define POSE_LOG_CLIENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <cabot_msgs/msg/pose_log.hpp>
#include <iostream>
#include <chrono>
#include <memory>

#include <InfluxDB.h>
#include <InfluxDBFactory.h>

using namespace std::chrono_literals;

class PoseLogListener : public rclcpp::Node{
public:
  PoseLogListener();

private:
  void pose_log_callback(const cabot_msgs::msg::PoseLog::SharedPtr msg);
  rclcpp::Subscription<cabot_msgs::msg::PoseLog>::SharedPtr subscription_;
};

#endif // POSE_LOG_CLIENT_HPP_
