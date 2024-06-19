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

ClientNode::ClientNode()
: Node("client_node"), throttle_(1.0)
{
  /*
  to debug multiple robot visualization on grafana, just use this line
  or set 'CABOT_INFLUXDB_ROBOT_NAME' to a comma separated names like
  "cabot1,cabot2,cabot3"
  self.robot_names = [f"cabot{i}" for i in range(1, 11)]
  */
  robot_name_ = this->declare_parameter<std::string>("robot_name", "");
  boost::algorithm::split(robot_names_, robot_name_, boost::is_any_of(","));
  for (std::vector<std::string>::const_iterator robot_name = robot_names_.begin();
    robot_name != robot_names_.end(); robot_name++)
  {
    const std::string & name = *robot_name;
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
  RCLCPP_INFO(
    this->get_logger(), "image_topics is %s, %s, %s",
    image_left_topic_.c_str(), image_center_topic_.c_str(), image_right_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Anchor file is %s", anchor_file_.c_str());
  if (!anchor_file_.empty()) {
    Anchor temp = get_anchor(anchor_file_);
    if (temp.lat != 0.0 || temp.lng != 0.0 || temp.rotate != 0.0) {
      anchor_ = temp;
    } else {
      RCLCPP_WARN(this->get_logger(), "Could not load anchor_file \"%s\"", anchor_file_.c_str());
    }
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
  image_left_throttle_ = std::make_shared<Throttle>(image_interval_);
  image_center_throttle_ = std::make_shared<Throttle>(image_interval_);
  image_right_throttle_ = std::make_shared<Throttle>(image_interval_);

  influxdb_ = influxdb::InfluxDBBuilder::http(host_ + "?db=" + bucket_)
    .setTimeout(std::chrono::seconds{5})
    .setAuthToken(token_)
    .setVerifyCertificate(false)
    .connect();

  pose_log_sub_ = this->create_subscription<cabot_msgs::msg::PoseLog>(
    "/cabot/pose_log", 10, [this](const cabot_msgs::msg::PoseLog::SharedPtr msg) {
      pose_log_throttle_->call(&ClientNode::pose_log_callback, this, msg);
    });
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cabot/cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
      cmd_vel_throttle_->call(&ClientNode::cmd_vel_callback, this, msg);
    });
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      odom_throttle_->call(&ClientNode::odom_callback, this, msg);
    });
  diag_agg_sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics_agg", 10, [this](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
      diag_agg_throttle_->call(&ClientNode::diagnostics_callback, this, msg);
    });
  plan_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path_all", 10, std::bind(&ClientNode::path_callback, this, std::placeholders::_1));
  battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
    battery_topic_, 10, [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
      battery_throttle_->call(&ClientNode::battery_callback, this, msg);
    });
  image_left_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    image_left_topic_, 10, [this](const sensor_msgs::msg::Image::SharedPtr msg) {
      image_left_throttle_->call(&ClientNode::image_callback, this, msg, "left");
    });
  image_center_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    image_center_topic_, 10, [this](const sensor_msgs::msg::Image::SharedPtr msg) {
      image_center_throttle_->call(&ClientNode::image_callback, this, msg, "center");
    });
  image_right_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    image_right_topic_, 10, [this](const sensor_msgs::msg::Image::SharedPtr msg) {
      image_right_throttle_->call(&ClientNode::image_callback, this, msg, "right");
    });
  event_sub_ = this->create_subscription<cabot_msgs::msg::Log>(
    "/cabot/activity_Log", 10,
    std::bind(&ClientNode::activity_log_callback, this, std::placeholders::_1));
}

void ClientNode::send_point(influxdb::Point && point)
{
  // prevent too much connection errors
  std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
  int64_t now_sec =
    std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
  if (last_error_ != 0 && now_sec - last_error_ < 1.0) {
    return;
  }
  try {
    influxdb_->write(std::move(point));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error writing to InfluxDB: %s", e.what());
    last_error_ = now_sec;
  }
}

std::chrono::time_point<std::chrono::system_clock> ClientNode::get_nanosec(
  const rclcpp::Time & stamp)
{
  try {
    rcl_clock_type_t clock_type = this->get_clock()->get_clock_type();
    rclcpp::Time adjusted_stamp = stamp;
    if (stamp.get_clock_type() != clock_type) {
      adjusted_stamp = rclcpp::Time(stamp.nanoseconds(), clock_type);
    }
    rclcpp::Time zero_time(0, 0, clock_type);
    if (adjusted_stamp == zero_time || adjusted_stamp.seconds() < 1e6) {
      return std::chrono::system_clock::now();
    }
    std::chrono::nanoseconds ns =
      std::chrono::nanoseconds(
      static_cast<uint64_t>(adjusted_stamp.seconds() * 1e9) + adjusted_stamp.nanoseconds());
    return std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>(ns);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get nanoseconds: %s", e.what());
    return std::chrono::time_point<std::chrono::system_clock>();
  }
}

void ClientNode::euler_from_quaternion(
  const geometry_msgs::msg::Quaternion & q, double & roll,
  double & pitch, double & yaw)
{
  double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
  double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  roll = std::atan2(sinr_cosp, cosr_cosp);
  double sinp = 2.0 * (q.w * q.y - q.z * q.x);
  if (fabs(sinp) >= 1.0) {
    pitch = std::copysign(M_PI / 2.0, sinp);
  } else {
    pitch = std::asin(sinp);
  }
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  yaw = std::atan2(siny_cosp, cosy_cosp);
}

void ClientNode::pose_log_callback(const cabot_msgs::msg::PoseLog::SharedPtr msg)
{
  try {
    double roll, pitch, yaw;
    euler_from_quaternion(msg->pose.orientation, roll, pitch, yaw);
    int count = 0;
    for (std::vector<std::string>::const_iterator robot_name = robot_names_.begin();
      robot_name != robot_names_.end(); robot_name++)
    {
      influxdb::Point point = influxdb::Point{"pose_data"}
      .addField("lat", msg->lat)     // lat + 0.0005 * count
      .addField("lng", msg->lng)     // lng + count * 0.0005
      .addField("floor", msg->floor)
      .addField("yaw", -anchor_.rotate - yaw / M_PI * 180)
      .addTag("robot_name", robot_name_)
      .setTimestamp(get_nanosec(rclcpp::Time()));
      send_point(std::move(point));
      count++;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get log pose: %s", e.what());
  }
}

void ClientNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  try {
    influxdb::Point point = influxdb::Point{"cmd_vel"}
    .addField("linear", msg->linear.x)
    .addField("angular", msg->angular.z)
    .addTag("robot_name", robot_name_)
    .setTimestamp(get_nanosec(rclcpp::Time()));
    send_point(std::move(point));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get cmd vel: %s", e.what());
  }
}

void ClientNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  try {
    influxdb::Point point = influxdb::Point{"odometry"}
    .addField("linear", msg->twist.twist.linear.x)
    .addField("angular", msg->twist.twist.angular.z)
    .addTag("robot_name", robot_name_)
    .setTimestamp(get_nanosec(msg->header.stamp));
    send_point(std::move(point));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get odom: %s", e.what());
  }
}

void ClientNode::diagnostics_callback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
{
  try {
    std::unordered_map<std::string, int> diagnostics;
    for (std::vector<diagnostic_msgs::msg::DiagnosticStatus>::const_iterator diagnosticstatus =
      msg->status.begin();
      diagnosticstatus != msg->status.end(); diagnosticstatus++)
    {
      const diagnostic_msgs::msg::DiagnosticStatus & state = *diagnosticstatus;
      std::vector<std::string> items;
      boost::algorithm::split(items, state.name, boost::is_any_of("/"));
      if (items.size() != 3) {
        continue;
      }
      std::string name = items[2];
      if (diagnostics.find(name) == diagnostics.end()) {
        diagnostics[name] = 0;
      }
      int level = static_cast<int>(state.level);
      if (diagnostics[name] < level) {
        diagnostics[name] = level;
      }
    }
    for (std::vector<std::string>::const_iterator robot_name = robot_names_.begin();
      robot_name != robot_names_.end(); robot_name++)
    {
      for (std::unordered_map<std::string, int>::const_iterator diagnostic = diagnostics.begin();
        diagnostic != diagnostics.end(); diagnostic++)
      {
        const std::string & name = diagnostic->first;
        int level = diagnostic->second;
        influxdb::Point point = influxdb::Point{"diagnostic"}
        .addField("level", level)
        .addTag("name", name)
        .addTag("robot_name", robot_name_)
        .setTimestamp(get_nanosec(rclcpp::Time()));
        send_point(std::move(point));
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get diagnostics: %s", e.what());
  }
}

void ClientNode::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
  try {
    std::chrono::time_point<std::chrono::system_clock> group = get_nanosec(rclcpp::Time());
    int64_t epoch =
      std::chrono::duration_cast<std::chrono::milliseconds>(group.time_since_epoch()).count();
    int count = 0;
    for (std::vector<std::string>::const_iterator robot_name = robot_names_.begin();
      robot_name != robot_names_.end(); robot_name++)
    {
      for (std::vector<geometry_msgs::msg::PoseStamped>::const_iterator pose = msg->poses.begin();
        pose != msg->poses.end(); pose++)
      {
        const geometry_msgs::msg::PoseStamped & pose_stamped = *pose;
        const geometry_msgs::msg::Point & position = pose_stamped.pose.position;
        Point localp = Point(position.x, position.y);
        Latlng globalp = local2global(localp, anchor_);
        std::string lat = std::to_string(globalp.lat);
        std::string lng = std::to_string(globalp.lng);
        influxdb::Point point = influxdb::Point{"plan"}
        .addField("lat", lat)     // lat + 0.0005 * count
        .addField("lng", lng)     // lng + count * 0.0005
        .addField("group", static_cast<int64_t>(epoch))
        .addTag("robot_name", robot_name_)
        .setTimestamp(get_nanosec(rclcpp::Time()));     // need to put point in different time
        send_point(std::move(point));
      }
      count++;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get path: %s", e.what());
  }
}

void ClientNode::battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  try {
    influxdb::Point point = influxdb::Point{"battery"}
    .addField("percentage", msg->percentage * 100.0)
    .addTag("robot_name", robot_name_)
    .setTimestamp(get_nanosec(msg->header.stamp));
    send_point(std::move(point));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get battery: %s", e.what());
  }
}

void ClientNode::image_callback(
  const sensor_msgs::msg::Image::SharedPtr msg,
  const std::string & direction)
{
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat cv_image = cv_ptr->image;
  std::vector<uchar> buf;
  cv::imencode(".jpg", cv_image, buf);
  std::string jpg_as_text = base64_encode(buf.data(), buf.size());
  try {
    influxdb::Point point = influxdb::Point{"image"}
    .addField("data", jpg_as_text)
    .addTag("format", "jpeg")
    .addTag("direction", direction)
    .addTag("robot_name", robot_name_)
    .setTimestamp(get_nanosec(msg->header.stamp));
    send_point(std::move(point));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get image: %s", e.what());
  }
}

std::string ClientNode::base64_encode(const unsigned char * data, size_t len)
{
  static const char encode_table[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string encoded;
  encoded.reserve(((len + 2) / 3) * 4);
  unsigned int temp = 0;
  int bits = 0;
  for (size_t i = 0; i < len; ++i) {
    temp = (temp << 8) + data[i];
    bits += 8;
    while (bits >= 6) {
      bits -= 6;
      encoded.push_back(encode_table[(temp >> bits) & 0x3F]);
    }
  }
  if (bits > 0) {
    temp <<= (6 - bits);
    encoded.push_back(encode_table[temp & 0x3F]);
  }
  while (encoded.size() % 4) {
    encoded.push_back('=');
  }
  return encoded;
}

void ClientNode::activity_log_callback(const cabot_msgs::msg::Log::SharedPtr msg)
{
  try {
    RCLCPP_INFO(this->get_logger(), "%s", msg->text.c_str());
    for (std::vector<std::string>::const_iterator robot_name = robot_names_.begin();
      robot_name != robot_names_.end(); robot_name++)
    {
      if (msg->category == "tour-text") {
        influxdb::Point point = influxdb::Point{"tour"}
        .addField("data", msg->text)
        .addTag("robot_name", robot_name_)
        .setTimestamp(get_nanosec(msg->header.stamp));
        send_point(std::move(point));
      }
      if (msg->category == "destination-text") {
        influxdb::Point point = influxdb::Point{"destination"}
        .addField("data", msg->text)
        .addTag("robot_name", robot_name_)
        .setTimestamp(get_nanosec(msg->header.stamp));
        send_point(std::move(point));
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get activity log: %s", e.what());
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<ClientNode> node_ = std::make_shared<ClientNode>();
  rclcpp::spin(node_);
  rclcpp::shutdown();
  return 0;
}
