#include "pose_log_client.hpp"

const std::string USERNAME = "cabot";
const std::string PASSWORD = "cabot-influxdb";
const std::string TOKEN = "";
const std::string ORGANIZATION_NAME = "cabot";
const std::string BUCKET_NAME = "grafana-test";
const std::string INFLUXDB_URL = "http://localhost:8086";


PoseLogListener::PoseLogListener() : Node("pose_log_listener"){
  subscription_ = create_subscription<cabot_msgs::msg::PoseLog>(
      "/cabot/pose_log", 10, std::bind(&PoseLogListener::pose_log_callback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Node is ready");
}

void PoseLogListener::pose_log_callback(const cabot_msgs::msg::PoseLog::SharedPtr msg){
  influxdb::Point point("pose_data");
  point.addField("lat", msg->lat);
  point.addField("lng", msg->lng);
  point.addField("floor", msg->floor);
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
  std::unique_ptr<PoseLogListener> node_ptr(std::make_unique<PoseLogListener>());
  std::shared_ptr<PoseLogListener> node(std::move(node_ptr));
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

