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
: Node("client_node"), throttle_(1.0),
  host_(this->declare_parameter<std::string>("host", "")),
  token_(this->declare_parameter<std::string>("token", "")),
  org_(this->declare_parameter<std::string>("org", "")),
  bucket_(this->declare_parameter<std::string>("bucket", "")),
  client_(host_, token_, org_, bucket_)
{
  robot_name_ = this->declare_parameter<std::string>("robot_name", "");
  anchor_file_ = this->declare_parameter<std::string>("anchor_file", "");
  battery_topic_ = this->declare_parameter<std::string>("battery_topic", "");
  image_left_topic_ = this->declare_parameter<std::string>("image_left_topic", "");
  image_center_topic_ = this->declare_parameter<std::string>("image_center_topic", "");
  image_right_topic_ = this->declare_parameter<std::string>("image_right_topic", "");
  rotate_image_ = this->declare_parameter<std::string>("image_rotate", "");
  rotate_images_ = split_string(rotate_image_, ',');
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
    "/path_all", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) {
      this->ClientNode::path_callback(msg);
    });
  battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
    battery_topic_, 10, [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
      battery_throttle_->call(&ClientNode::battery_callback, this, msg);
    });
  if(!image_left_topic_.empty()){
    image_left_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_left_topic_, 10, [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        image_left_throttle_->call(&ClientNode::image_callback, this, msg, "left");
      });
  }
  image_center_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    image_center_topic_, 10, [this](const sensor_msgs::msg::Image::SharedPtr msg) {
      image_center_throttle_->call(&ClientNode::image_callback, this, msg, "center");
    });
  if(!image_right_topic_.empty()){
    image_right_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_right_topic_, 10, [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        image_right_throttle_->call(&ClientNode::image_callback, this, msg, "right");
      });
  }
  event_sub_ = this->create_subscription<cabot_msgs::msg::Log>(
    "/cabot/activity_log", 10, [this](const cabot_msgs::msg::Log::SharedPtr msg) {
      this->ClientNode::activity_log_callback(msg);
    });
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

    InfluxPoint point{"pose_data"};
    point.addField("lat", msg->lat)
         .addField("lng", msg->lng)
         .addField("floor", msg->floor)
         .addField("yaw", -anchor_.rotate - yaw / M_PI * 180)
         .addTag("robot_name", robot_name_)
         .setTimestamp(get_nanosec(rclcpp::Time()));
    std::string lineProtocol = point.toLineProtocol();
    if (!client_.sendData(lineProtocol)) {
      throw std::runtime_error("Failed to send data to InfluxDB");
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get log pose: %s", e.what());
  }
}

void ClientNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  try {
    InfluxPoint point{"cmd_vel"};
    point.addField("linear", msg->linear.x)
         .addField("angular", msg->angular.z)
         .addTag("robot_name", robot_name_)
         .setTimestamp(get_nanosec(rclcpp::Time()));
    std::string lineProtocol = point.toLineProtocol();
    if (!client_.sendData(lineProtocol)) {
      throw std::runtime_error("Failed to send data to InfluxDB");
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get cmd vel: %s", e.what());
  }
}

void ClientNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  try {
    InfluxPoint point{"odometry"};
    point.addField("linear", msg->twist.twist.linear.x)
         .addField("angular", msg->twist.twist.angular.z)
         .addTag("robot_name", robot_name_)
         .setTimestamp(get_nanosec(msg->header.stamp));
    std::string lineProtocol = point.toLineProtocol();
    if (!client_.sendData(lineProtocol)) {
      throw std::runtime_error("Failed to send data to InfluxDB");
    }
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
    for (std::unordered_map<std::string, int>::const_iterator diagnostic = diagnostics.begin();
      diagnostic != diagnostics.end(); diagnostic++)
    {
      const std::string & name = diagnostic->first;
      int level = diagnostic->second;
      InfluxPoint point{"diagnostic"};
      point.addField("level", level)
           .addTag("name", name)
           .addTag("robot_name", robot_name_)
           .setTimestamp(get_nanosec(rclcpp::Time()));
      std::string lineProtocol = point.toLineProtocol();
      if (!client_.sendData(lineProtocol)) {
        throw std::runtime_error("Failed to send data to InfluxDB");
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
    for (std::vector<geometry_msgs::msg::PoseStamped>::const_iterator pose = msg->poses.begin();
      pose != msg->poses.end(); pose++)
    {
      const geometry_msgs::msg::PoseStamped & pose_stamped = *pose;
      const geometry_msgs::msg::Point & position = pose_stamped.pose.position;
      Point localp = Point(position.x, position.y);
      Latlng globalp = local2global(localp, anchor_);
      std::string lat = std::to_string(globalp.lat);
      std::string lng = std::to_string(globalp.lng);
      InfluxPoint point{"plan"};
      point.addField("lat", globalp.lat) // Use string format of lat on localhost
           .addField("lng", globalp.lng) // use string format of lng on localhost
           .addField("group", static_cast<int64_t>(epoch))
           .addTag("robot_name", robot_name_)
           .setTimestamp(get_nanosec(rclcpp::Time()));     // need to put point in different time
      std::string lineProtocol = point.toLineProtocol();
      if (!client_.sendData(lineProtocol)) {
        throw std::runtime_error("Failed to send data to InfluxDB");
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get path: %s", e.what());
  }
}

void ClientNode::battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  try {
    InfluxPoint point{"battery"};
    point.addField("percentage", msg->percentage * 100.0)
         .addTag("robot_name", robot_name_)
         .setTimestamp(get_nanosec(msg->header.stamp));
    std::string lineProtocol = point.toLineProtocol();
    if (!client_.sendData(lineProtocol)) {
      throw std::runtime_error("Failed to send data to InfluxDB");
    }
    //RCLCPP_INFO(this->get_logger(), "get battery: %.2f", msg->percentage);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get battery: %s", e.what());
  }
}

void ClientNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg, const std::string & direction)
{
  try {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat resized_image = resize_with_aspect_ratio(cv_ptr->image, 512);
    cv::Mat rotated_image = rotate_image(resized_image, direction);
    std::vector<uchar> buf;
    cv::imencode(".jpg", rotated_image, buf, {cv::IMWRITE_JPEG_QUALITY, 80});
    std::string jpg_as_text = base64_encode(buf);
    InfluxPoint point{"image"};
    point.addField("data", jpg_as_text)
         .addTag("format", "jpeg")
         .addTag("direction", direction)
         .addTag("robot_name", robot_name_)
         .setTimestamp(get_nanosec(msg->header.stamp));
    std::string lineProtocol = point.toLineProtocol();
    if (!client_.sendData(lineProtocol)) {
      throw std::runtime_error("Failed to send data to InfluxDB");
    }
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to process image: %s", e.what());
  }
}

cv::Mat ClientNode::resize_with_aspect_ratio(const cv::Mat& image, int target_size) {
  int original_width = image.cols;
  int original_height = image.rows;
  int new_width, new_height;
    if (original_width > original_height) {
      new_width = target_size;
      new_height = static_cast<int>((static_cast<double>(target_size) / original_width) * original_height);
    } else {
      new_height = target_size;
      new_width = static_cast<int>((static_cast<double>(target_size) / original_height) * original_width);
    }
  cv::Mat resized_image;
  cv::resize(image, resized_image, cv::Size(new_width, new_height), 0, 0, cv::INTER_AREA);
  return resized_image;
}

cv::Mat ClientNode::rotate_image(const cv::Mat& image, const std::string& direction) {
  bool rotate = false;
  for (const auto& dir : this->rotate_images_) {
    if (dir ==direction) {
      rotate = true;
      break;
    }
  }
  cv::Mat rotated_image = image;
  if (rotate) {
    cv::rotate(image, rotated_image, cv::ROTATE_180);
  }
  return rotated_image;
}

std::vector<std::string> ClientNode::split_string(const std::string &str, char delimiter) {
  std::vector<std::string> tokens;
  std::stringstream ss(str);
  std::string item;
  while (std::getline(ss, item, delimiter)) {
    tokens.push_back(item);
  }
  return tokens;
}

std::string ClientNode::base64_encode(const std::vector<uchar>& data)
{
  static const char encode_table[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string encoded;
  encoded.reserve(((data.size() + 2) / 3) * 4);
  unsigned int temp = 0;
  int bits = 0;
  for (size_t i = 0; i < data.size(); ++i) {
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
    RCLCPP_INFO(
      this->get_logger(),
      "header_stamp(sec=%d, nanosec=%d, frame_id=%s), category=%s, text=%s, memo=%s",
      msg->header.stamp.sec, msg->header.stamp.nanosec, msg->header.frame_id.c_str(),
      msg->category.c_str(), msg->text.c_str(), msg->memo.c_str());
    if (msg->category == "tour-text") {

      InfluxPoint point{"tour"};
      point.addField("data", msg->text)
           .addTag("robot_name", robot_name_)
           .setTimestamp(get_nanosec(msg->header.stamp));
      try {
          std::string lineProtocol = point.toLineProtocol();
          if (!client_.sendData(lineProtocol)) {
              throw std::runtime_error("Failed to send data to InfluxDB");
          }
      } catch (const std::exception& e) {
          RCLCPP_ERROR(this->get_logger(), "Failed to get activity log: %s", e.what());
      }
    }
    if (msg->category == "destination-text") {
      InfluxPoint point{"destination"};
      point.addField("data", msg->text)
           .addTag("robot_name", robot_name_)
           .setTimestamp(get_nanosec(msg->header.stamp));
      try {
          std::string lineProtocol = point.toLineProtocol();
          if (!client_.sendData(lineProtocol)) {
              throw std::runtime_error("Failed to send data to InfluxDB");
          }
      } catch (const std::exception& e) {
          RCLCPP_ERROR(this->get_logger(), "Failed to get activity log: %s", e.what());
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
