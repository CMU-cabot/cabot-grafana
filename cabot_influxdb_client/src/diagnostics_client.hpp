#ifndef DIAGNOSTICS_CLIENT_HPP_
#define DIAGNOSTICS_CLIENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <iostream>
#include <chrono>
#include <memory>
#include <sstream>
#include <string>

#include <InfluxDB.h>
#include <InfluxDBFactory.h>

using namespace std::chrono_literals;

class DiagnosticsListener : public rclcpp::Node{
public:
  DiagnosticsListener();

private:
  void diagnostics_callback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);
  void diagnostics2_callback(const diagnostic_msgs::msg::DiagnosticStatus::SharedPtr msg);

  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr subscription_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr subscription2_;
};

#endif // DIAGNOSTICS_CLIENT_HPP_
