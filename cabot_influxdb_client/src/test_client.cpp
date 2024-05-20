#include "test_client.hpp"

const std::string USERNAME = "cabot";
const std::string PASSWORD = "cabot-influxdb";
const std::string TOKEN = "";
const std::string ORGANIZATION_NAME = "cabot";
const std::string BUCKET_NAME = "grafana-test";
const std::string INFLUXDB_URL = "http://localhost:8086";


TemperatureSubscriber::TemperatureSubscriber() : Node("temperature_subscriber"){
  subscription_ = create_subscription<sensor_msgs::msg::Temperature>(
      "/temperature", 10, std::bind(&TemperatureSubscriber::listener_callback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Node is ready");
}

void TemperatureSubscriber::listener_callback(const sensor_msgs::msg::Temperature::SharedPtr msg){
  RCLCPP_INFO(this->get_logger(), "Received temperature: %.2f", msg->temperature);

  influxdb::Point point("temperature");
  point.addTag("unit", "celsius");
  point.addField("value", msg->temperature);
  point.setTimestamp(std::chrono::system_clock::now());

  try{
    std::unique_ptr<influxdb::InfluxDB> idb(influxdb::InfluxDBFactory::Get(INFLUXDB_URL));
    idb->write(std::move(point));
  }catch(const std::exception& e){
    RCLCPP_ERROR(this->get_logger(), "Error writing to InfluxDB: %s", e.what());
  }
}

int main(int argc, char* argv[]){
  rclcpp::init(argc, argv);
  std::unique_ptr<TemperatureSubscriber> node_ptr(std::make_unique<TemperatureSubscriber>());
  std::shared_ptr<TemperatureSubscriber> node(std::move(node_ptr));
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

