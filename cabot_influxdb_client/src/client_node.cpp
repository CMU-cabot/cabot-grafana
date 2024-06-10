#include "client_node.hpp"

ClientNode::ClientNode() : Node("client_node"), throttle_(1.0){
  // CaBotRclcppUtil::initialize(this);
   
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
  // reading anchor
  /*
  if(!anchor_file_.empty()){
    anchor_ = geoutil::get_anchor(anchor_file_);
    if(anchor_.is_valid()){
      logger_.info(F"anchor file is {anchor_file_}");
    }else{
      logger_.warn(F"Failed to load anchor file is {anchor_file_}");
    }
  }
  */

  influxdb_ = influxdb::InfluxDBBuilder::http(host_ + "?db=" + bucket_)
                  .setTimeout(std::chrono::seconds{5})
                  .setAuthToken(token_)
                  .setVerifyCertificate(false)
                  .connect();
  RCLCPP_INFO(this->get_logger(), "URL: %s", influxdb_.get());
  
  pose_log_sub_ = this->create_subscription<cabot_msgs::msg::PoseLog>(
    "/cabot/pose_log", 10, std::bind(&ClientNode::pose_log_callback, this, std::placeholders::_1));
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cabot/cmd_vel", 10, std::bind(&ClientNode::cmd_vel_callback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10, std::bind(&ClientNode::odom_callback, this, std::placeholders::_1));
  diag_agg_sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics_agg", 10, std::bind(&ClientNode::diagnostics_callback, this, std::placeholders::_1));
  plan_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path_all",10, std::bind(&ClientNode::path_callback, this, std::placeholders::_1));
  battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
    battery_topic_, 10, std::bind(&ClientNode::battery_callback, this, std::placeholders::_1));
  image_left_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    image_left_topic_, 10, [this] (const sensor_msgs::msg::Image::SharedPtr msg) { image_callback(msg, "left"); });
  image_center_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    image_center_topic_, 10, [this] (const sensor_msgs::msg::Image::SharedPtr msg) { image_callback(msg, "center"); });
  image_right_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    image_right_topic_, 10, [this] (const sensor_msgs::msg::Image::SharedPtr msg) { image_callback(msg, "right"); });
  event_sub_ = this->create_subscription<cabot_msgs::msg::Log>(
    "/cabot/activity_Log", 10, std::bind(&ClientNode::activity_log_callback, this, std::placeholders::_1));

  // bridge_ = std::make_shared<cv_bridge::CvBridge>();
  RCLCPP_INFO(this->get_logger(), "InfluxDB Host: %s", host_.c_str());
}

/*
void ClientNode::send_point(const influxdb::Point point){
  // prevent too much connection errors
  if(last_error_ && std::time(0) - last_error_ < 1.0){
    return;
  }
  try{
    std::unique_ptr<influxdb::InfluxDB> idb(influxdb::InfluxDBFactory::Get(INFLUXDB_URL));
    idb->write(std::move(point));
  }catch(const std::exception& e){
    RCLCPP_ERROR(this->get_logger(), "Error writing to InfluxDB: %s", e.what());
    last_error_ = time.time();
  }
}
*/

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
        .addField("lat", msg->lat + 0.0005 * count)
        .addField("lng", msg->lng + 0.0005 * count)
        .addField("floor", msg->floor)
        .addField("yaw", /*- anchor_.rotate*/ - yaw / M_PI * 180)
        .addTag("robot_name", robot_name_)
        .setTimestamp(get_nanosec(rclcpp::Time()));
    influxdb_->write(std::move(point));
    count++;
  }
}

void ClientNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
  influxdb::Point point = influxdb::Point{"cmd_vel"}
      .addField("linear", msg->linear.x)
      .addField("angular", msg->angular.z)
      .addTag("robot_name", robot_name_)
      .setTimestamp(get_nanosec(rclcpp::Time()));
  influxdb_->write(std::move(point));
}

void ClientNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
  influxdb::Point point = influxdb::Point{"odometry"}
      .addField("linear", msg->twist.twist.linear.x)
      .addField("angular", msg->twist.twist.linear.x)
      .addTag("robot_name", robot_name_)
      .setTimestamp(get_nanosec(rclcpp::Time()));
  influxdb_->write(std::move(point));
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
  throttle_.call([this, msg](){
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
        influxdb_->write(std::move(point));
      }
    }
  });
}

void ClientNode::path_callback(const nav_msgs::msg::Path::SharedPtr msg){
  auto group = get_nanosec(rclcpp::Time());
  int count = 0;
  /*
  for(const auto& robot_name : robot_name_){
    for(const auto& pose_stamped : msg->poses){
      const auto& pose_stamped.pose.postion;
      influxdb::Point localp(position.x, position.y);
      GlobalPoint globalp = local2global(localp, anchor_);
      influxdb::Point point = influxdb::Point{"plan"}
          .addField("lat", globalp.lat + 0.0005 * count)
          .addField("lng", globalp.lng + 0.0005 * count)
          .addField("group", group)
          .addTag("robot_name", robot_name_)
          .setTimestamp(get_nanosec(pose_stamped.header.stamp));
      influxdb_->write(std::move(point));
    }
    count++;
  }
  */
}

void ClientNode::battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg){
  influxdb::Point point = influxdb::Point{"battery"}
      .addField("percentage", msg->percentage * 100.0)
      .addTag("robot_name", robot_name_)
      .setTimestamp(get_nanosec(rclcpp::Time()));
  influxdb_->write(std::move(point));
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
  influxdb_->write(std::move(point));
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
      influxdb_->write(std::move(point));
    }
    if(msg->category == "destination-text"){
      influxdb::Point point = influxdb::Point{"destination"}
          .addField("data", msg->text)
          .addTag("robot_name", robot_name_)
          .setTimestamp(get_nanosec(rclcpp::Time()));
      influxdb_->write(std::move(point));
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
