#include "diagnostics_client.hpp"

const std::string USERNAME = "cabot";
const std::string PASSWORD = "cabot-influxdb";
const std::string TOKEN = "";
const std::string ORGANIZATION_NAME = "cabot";
const std::string BUCKET_NAME = "grafana-test";
const std::string INFLUXDB_URL = "http://localhost:8086";

std::string getLevelText(int level){
  switch(level){
    case diagnostic_msgs::msg::DiagnosticStatus::OK:
      return "OK";
    case diagnostic_msgs::msg::DiagnosticStatus::WARN:
      return "WARN";
    case diagnostic_msgs::msg::DiagnosticStatus::ERROR:
      return "ERROR";
    case diagnostic_msgs::msg::DiagnosticStatus::STALE:
      return "STALE";
    default:
      return "UNKNOWN";
  }
}

DiagnosticsListener::DiagnosticsListener() : Node("diagnostics_listener"){
  subscription_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics_agg", 10, std::bind(&DiagnosticsListener::diagnostics_callback, this, std::placeholders::_1));
  subscription2_ = create_subscription<diagnostic_msgs::msg::DiagnosticStatus>(
      "/diagnostics_toplevel_state", 10, std::bind(&DiagnosticsListener::diagnostics2_callback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Node is ready");
}

void DiagnosticsListener::diagnostics_callback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg){
  const std::string target_name = "Soft: Navigation2";
  int max_level = 0;
  for(const diagnostic_msgs::msg::DiagnosticStatus& state : msg->status){
    std::vector<std::string> items;
    std::stringstream ss(state.name);
    std::string item;
    while(std::getline(ss, item, '/')){
      items.push_back(item);
    }
    if(items.size() < 3){
      continue;
    }
    if(items[2] == target_name){
      int level = static_cast<int>(state.level);
      max_level = std::max(max_level, level);
    }
  }

  const std::string level_text = getLevelText(max_level);
  influxdb::Point point("diagnostic");
  point.addTag("name", target_name);
  point.addField("level", max_level);
  point.addField("level_text", level_text);
  point.setTimestamp(std::chrono::system_clock::now());

  try{
    std::unique_ptr<influxdb::InfluxDB> idb(influxdb::InfluxDBFactory::Get(INFLUXDB_URL));
    idb->write(std::move(point));
  }catch(const std::exception& e){
    RCLCPP_ERROR(this->get_logger(), "Error writing to InfluxDB: %s", e.what());
  }
}

void DiagnosticsListener::diagnostics2_callback(const diagnostic_msgs::msg::DiagnosticStatus::SharedPtr msg){
  // unimplemented
}

int main(int argc, char* argv[]){
  rclcpp::init(argc, argv);
  std::unique_ptr<DiagnosticsListener> node_ptr(std::make_unique<DiagnosticsListener>());
  std::shared_ptr<DiagnosticsListener> node(std::move(node_ptr));
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
