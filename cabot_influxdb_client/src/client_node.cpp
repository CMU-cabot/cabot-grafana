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

#include "client_node.hpp"
#include "geoutil.hpp"

ClientNode::ClientNode() : Node("client_node"), throttle_(1.0){
  /*
  to debug multiple robot visualization on grafana, just use this line
  or set 'CABOT_INFLUXDB_ROBOT_NAME' to a comma separated names like
  "cabot1,cabot2,cabot3"
  self.robot_names = [f"cabot{i}" for i in range(1, 11)]
  */   
  robot_name_ = this->declare_parameter<std::string>("robot_name", "");
  boost::algorithm::split(robot_names_, robot_name_, boost::is_any_of(","));
  for(const std::string& name : robot_names_){
    std::cout << name << std::endl;
  }
  host_ = this->declare_parameter<std::string>("host", "");
  token_ = this->declare_parameter<std::string>("token", "");
  org_ = this->declare_parameter<std::string>("org", "");
  bucket_ = this->declare_parameter<std::string>("bucket", "");
  anchor_file_ = this->declare_parameter<std::string>("anchor_file", "");
  battery_topic_ = this->declare_parameter<std::string>("battery_topic", "");
  image_left_topic_ = this->declare_parameter<std::string>("image_left_topic", "");
  image_center_topic_ = this->declare_parameter<std::string>("image_center_topic", "");
  image_right_topic_ = this->declare_parameter<std::string>("image_right_topic", "");
  if(!anchor_file_.empty()){
    anchor_ = get_anchor(anchor_file_);
    /*
    if(anchor_.is_valid()){
      // logger_.info(F"anchor file is {anchor_file_}");
    }else{
      // logger_.warn(F"Failed to load anchor file is {anchor_file_}");
    }*/
  }
  pose_interval_ = this->declare_parameter<double>("pose_interval", 1.0);
  cmd_vel_interval_ = this->declare_parameter<double>("cmd_vel_interval", 0.2);
  odom_interval_ = this->declare_parameter<double>("odom_interval", 0.2);
  diag_agg_interval_ = this->declare_parameter<double>("diag_agg_interval", 1.0);
  battery_interval_ = this->declare_parameter<double>("battery_interval", 1.0);
  image_interval_ = this->declare_parameter<double>("image_interval", 5.0);
  pose_log_throttle_ = std::make_shared<Throttle>(pose_interval_);
  cmd_vel_throttle_ = std::make_shared<Throttle>(cmd_vel_interval_);
  odom_throttle_ = std::make_shared<Throttle>(odom_interval_);
  diag_agg_throttle_ = std::make_shared<Throttle>(diag_agg_interval_);
  battery_throttle_ = std::make_shared<Throttle>(battery_interval_);
  image_throttle_ = std::make_shared<Throttle>(image_interval_);

  influxdb_ = influxdb::InfluxDBBuilder::http(host_ + "?db=" + bucket_)
                  .setTimeout(std::chrono::seconds{5})
                  .setAuthToken(token_)
                  .setVerifyCertificate(false)
                  .connect();
  
  pose_log_sub_ = this->create_subscription<cabot_msgs::msg::PoseLog>(
    "/cabot/pose_log", 10, [this](const cabot_msgs::msg::PoseLog::SharedPtr msg) { 
      pose_log_throttle_->call(&ClientNode::pose_log_callback, this, msg); });
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cabot/cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
      cmd_vel_throttle_->call(&ClientNode::cmd_vel_callback, this, msg); });
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { 
      odom_throttle_->call(&ClientNode::odom_callback, this, msg); });
  diag_agg_sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics_agg", 10, [this](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
      diag_agg_throttle_->call(&ClientNode::diagnostics_callback, this, msg); });
  plan_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path_all",10, std::bind(&ClientNode::path_callback, this, std::placeholders::_1));
  battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
    battery_topic_, 10, [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
      battery_throttle_->call(&ClientNode::battery_callback, this, msg); });
  image_left_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    image_left_topic_, 10, [this] (const sensor_msgs::msg::Image::SharedPtr msg) {
      image_throttle_->call(&ClientNode::image_callback, this, msg, "left"); });
  image_center_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    image_center_topic_, 10, [this] (const sensor_msgs::msg::Image::SharedPtr msg) {
      image_throttle_->call(&ClientNode::image_callback, this, msg, "center"); });
  image_right_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    image_right_topic_, 10, [this] (const sensor_msgs::msg::Image::SharedPtr msg) {
      image_throttle_->call(&ClientNode::image_callback, this, msg, "right"); });
  event_sub_ = this->create_subscription<cabot_msgs::msg::Log>(
    "/cabot/activity_Log", 10, std::bind(&ClientNode::activity_log_callback, this, std::placeholders::_1));

  // bridge_ = std::make_shared<cv_bridge::CvBridge>();
  RCLCPP_INFO(this->get_logger(), "InfluxDB Host: %s", host_.c_str());
}

void ClientNode::send_point(influxdb::Point&& point){
  // prevent too much connection errors
  auto now = std::chrono::system_clock::now();
  auto now_sec = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
  if(last_error_ != 0 && now_sec - last_error_ < 1.0){
    return;
  }
  try{
    influxdb_->write(std::move(point));
  }catch(const std::exception& e){
    RCLCPP_ERROR(this->get_logger(), "Error writing to InfluxDB: %s", e.what());
    last_error_ = now_sec;
  }
}


std::chrono::time_point<std::chrono::system_clock> ClientNode::get_nanosec(const rclcpp::Time& stamp){
  using namespace std::chrono;

  if(stamp == rclcpp::Time(0,0) || stamp.seconds() < 1e6){
    return system_clock::now();
  }
  auto ns =nanoseconds(static_cast<uint64_t>(stamp.seconds() * 1e9) + stamp.nanoseconds());
  return time_point<system_clock, nanoseconds>(ns);
}


void ClientNode::euler_from_quaternion(const geometry_msgs::msg::Quaternion& q, double& roll, double& pitch, double& yaw){
  double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
  double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
  roll = std::atan2(sinr_cosp, cosr_cosp);

  double sinp = 2 * (q.w * q.y - q.z * q.x);
  if(std::abs(sinp) >= 1){
    pitch = std::copysign(M_PI / 2, sinp); 
  }else{
    pitch = std::asin(sinp);
  }
  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  yaw = std::atan2(siny_cosp, cosy_cosp);
}

void ClientNode::pose_log_callback(const cabot_msgs::msg::PoseLog::SharedPtr msg){ 
  double roll, pitch, yaw;
  euler_from_quaternion(msg->pose.orientation, roll, pitch, yaw);
  int count = 0;
  for(const auto& robot_name : robot_name_){
    influxdb::Point point = influxdb::Point{"pose_data"}
        .addField("lat", msg->lat /*+ 0.0005 * count*/)
        .addField("lng", msg->lng /*+ 0.0005 * count*/)
        .addField("floor", msg->floor)
        .addField("yaw", - anchor_.rotate - yaw / M_PI * 180)
        .addTag("robot_name", robot_name_)
        .setTimestamp(get_nanosec(rclcpp::Time()));
    send_point(std::move(point));
    count++;
  }
}

void ClientNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
  influxdb::Point point = influxdb::Point{"cmd_vel"}
      .addField("linear", msg->linear.x)
      .addField("angular", msg->angular.z)
      .addTag("robot_name", robot_name_)
      .setTimestamp(get_nanosec(rclcpp::Time()));
  send_point(std::move(point));
}

void ClientNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
  influxdb::Point point = influxdb::Point{"odometry"}
      .addField("linear", msg->twist.twist.linear.x)
      .addField("angular", msg->twist.twist.linear.x)
      .addTag("robot_name", robot_name_)
      .setTimestamp(get_nanosec(rclcpp::Time()));
  send_point(std::move(point));
}

std::vector<std::string> ClientNode::split(const std::string& str, char delimiter){
  std::vector<std::string> tokens;
  std::string token;
  std::istringstream tokenStream(str);
  while(std::getline(tokenStream, token, delimiter)){
    tokens.push_back(token);
  }
  return tokens;
}

void ClientNode::diagnostics_callback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg){
  std::unordered_map<std::string, int> diagnostics;
  for(const auto& state : msg->status){
  auto items = split(state.name, '/');
    if(items.size() != 3){
      continue;
    }
    std::string name = items[2];
    if(diagnostics.find(name) == diagnostics.end()){
      diagnostics[name] = 0;
    }
    int level = static_cast<int>(state.level);
    if(diagnostics[name] < level){
      diagnostics[name] = level;
    }
  }
  for(const auto& robot_name : robot_name_){
    for(const auto& [name, level] : diagnostics){
      influxdb::Point point = influxdb::Point{"diagnostic"}
          .addField("level", level)
          .addTag("name", name)
          .addTag("robot_name", robot_name_)
          .setTimestamp(get_nanosec(rclcpp::Time()));
      send_point(std::move(point));
    }
  }
}

void ClientNode::path_callback(const nav_msgs::msg::Path::SharedPtr msg){
  auto group = get_nanosec(rclcpp::Time());
  auto epoch = std::chrono::duration_cast<std::chrono::milliseconds>(group.time_since_epoch()).count();
  int count = 0;
  for(const auto& robot_name : robot_name_){
    for(const auto& pose_stamped : msg->poses){
      const auto& position = pose_stamped.pose.position;
      auto localp = Point(position.x, position.y);
      //Latlng globalp = local2global(localp, anchor_); 
      influxdb::Point point = influxdb::Point{"plan"}
          .addField("lat", "35.619135") //incomplete
          .addField("lng", "139.7768801") //incomplete
          .addField("group", static_cast<int64_t>(epoch))
          .addTag("robot_name", robot_name_)
	  .setTimestamp(get_nanosec(rclcpp::Time())); // need to put point in different time
      send_point(std::move(point));
    }
    count++;
  }
}

void ClientNode::battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg){
  influxdb::Point point = influxdb::Point{"battery"}
      .addField("percentage", msg->percentage * 100.0)
      .addTag("robot_name", robot_name_)
      .setTimestamp(get_nanosec(rclcpp::Time()));
  send_point(std::move(point));
}

void ClientNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg, const std::string& direction){
  cv_bridge::CvImagePtr cv_ptr;
  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }catch(cv_bridge::Exception& e){
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat cv_image = cv_ptr->image;
  std::vector<uchar> buf;
  cv::imencode(".jpg", cv_image, buf);
  std::string jpg_as_text = base64_encode(buf.data(), buf.size());
  influxdb::Point point = influxdb::Point{"image"}
      .addField("data", jpg_as_text)
      .addTag("format", "jpeg")
      .addTag("direction", direction)
      .addTag("robot_name", robot_name_)
      .setTimestamp(get_nanosec(rclcpp::Time()));
  send_point(std::move(point));
}

std::string ClientNode::base64_encode(const unsigned char *data, size_t len){
  static const char encode_table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string encoded;
  encoded.reserve(((len / 3) + (len % 3 > 0)) * 4);
  unsigned int temp = 0;
  for(size_t i = 0; i < len; i++){
    temp = (temp << 8) + data[i];
    if ((i % 3) == 2){
      encoded.push_back(encode_table[(temp & 0x00FC0000) >> 18]);
      encoded.push_back(encode_table[(temp & 0x0003F000) >> 12]);
      encoded.push_back(encode_table[(temp & 0x00000FC0) >> 6]);
      encoded.push_back(encode_table[temp & 0x0000003F]);
      temp = 0;
    }
  }
  switch(len % 3){
    case 1:
      temp <<= 16;
      encoded.push_back(encode_table[(temp & 0x00FC0000) >> 18]);
      encoded.push_back(encode_table[(temp & 0x0003F000) >> 12]);
      encoded.push_back('=');
      encoded.push_back('=');
      break;
    case 2:
      temp <<= 8;
      encoded.push_back(encode_table[(temp & 0x00FC0000) >> 18]);
      encoded.push_back(encode_table[(temp & 0x0003F000) >> 12]);
      encoded.push_back(encode_table[(temp & 0x00000FC0) >> 6]);
      encoded.push_back('=');
      break;
  }
  return encoded;
}

void ClientNode::activity_log_callback(const cabot_msgs::msg::Log::SharedPtr msg){
  //logger
  for(const auto& robot_name : robot_name_){
    if(msg->category == "tour-text"){
      influxdb::Point point = influxdb::Point{"tour"}
          .addField("data", msg->text)
          .addTag("robot_name", robot_name_)
          .setTimestamp(get_nanosec(rclcpp::Time()));
      send_point(std::move(point));
    }
    if(msg->category == "destination-text"){
      influxdb::Point point = influxdb::Point{"destination"}
          .addField("data", msg->text)
          .addTag("robot_name", robot_name_)
          .setTimestamp(get_nanosec(rclcpp::Time()));
      send_point(std::move(point));
    }
  }
}

int main(int argc, char ** argv){
  rclcpp::init(argc, argv);
  auto node_ = std::make_shared<ClientNode>();
  rclcpp::spin(node_);
  rclcpp::shutdown();
  return 0;
}
