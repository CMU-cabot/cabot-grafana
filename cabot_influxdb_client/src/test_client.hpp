#ifndef TEST_CLIENT_HPP_
#define TEST_CLIENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <InfluxDB.h>
#include <InfluxDBFactory.h>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class TemperatureSubscriber : public rclcpp::Node{
public:
  TemperatureSubscriber();

private:
  void listener_callback(const sensor_msgs::msg::Temperature::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr subscription_;
};

#endif // TEST_CLIENT_HPP_
