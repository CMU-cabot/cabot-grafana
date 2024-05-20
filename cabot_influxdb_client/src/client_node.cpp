#include "client_node.hpp"
/*
ClientNode::ClientNode() : Node("client_node"){
  // CaBotRclcppUtil::initialize(this);
  robot_name_param = declare_parameter("robot_name", "");
  robot_name_ = robot_name_param.get<std::string>();
  rclcpp::Parameter param_robot_names = declare_parameter("CABOT_INFLUXDB_ROBOT_NAME", "");

  host_ = declare_parameter("host", "").value;
  token_ = declare_parameter("token", "").value;
  org_ = declare_parameter("org", "").value;
  bucket_ = declare_parameter("bucket", "").value;
  anchor_file_ = declare_parameter("anchor_file", "").value;
  battery_topic_ = declare_parameter("battery_topic", "").value;
  image_left_topic_ = declare_parameter("image_left_topic", "").value;
  image_center_topic_ = declare_parameter("image_center_topic", "").value;
  image_right_topic_ = declare_parameter("image_right_topic", "").value;

  // reading anchor
  if(!anchor_file_.empty()){
    anchor_ = geoutil::get_anchor(anchor_file_);
    if(anchor_.is_valid()){
      logger_.info(F"anchor file is {anchor_file_}");
    }else{
      logger_.warn(F"Failed to load anchor file is {anchor_file_}");
    }
  }

  // initialize client
  // client_ = std::make_shared<influxdb_client::InfluxDBClient>(url=host_, token=token_, org=org_);
  // write_api_ = client_->write_api(SYNCHRONOUS);
  // logger_ = get_logger();

  ClientNode::pose_log_sub_ = create_subscription<cabot_msgs::msg::PoseLog>("/cabot/pose_log",10,
      std::bind(&ClientNode::pose_log_callback, this, std::placeholders::_1, 1.0));
  ClientNode::cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>("/cabot/cmd_vel",10,
      std::bind(&ClientNode::cmd_vel_callback, this, std::placeholders::_1, 0.2));
  ClientNode::odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom",10,
      std::bind(&ClientNode::odom_callback, this, std::placeholders::_1, 0.2));
  ClientNode::diag_agg_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics_agg",10,
      std::bind(&ClientNode::diagnostics_callback, this, std::placeholders::_1, 1.0));
  ClientNode::plan_sub_ = create_subscription<nav_msgs::msg::Path>("/path_all",10,
      std::bind(&ClientNode::path_callback, this, std::placeholders::_1));
  ClientNode::battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(battery_topic_,10,
      std::bind(&ClientNode::battery_callback, this, std::placeholders::_1, 1.0));
  ClientNode::image_left_sub_ = create_subscription<sensor_msgs::msg::Image>(image_left_topic_,10,
      std::bind(&ClientNode::image_callback, this, std::placeholders::_1, 0.2, "left"));
  ClientNode::image_center_sub_ = create_subscription<sensor_msgs::msg::Image>(image_center_topic_,10,
      std::bind(&ClientNode::image_callback, this, std::placeholders::_1, 0.2, "center"));
  ClientNode::image_right_sub_ = create_subscription<sensor_msgs::msg::Image>(image_right_topic_,10,
      std::bind(&ClientNode::image_callback, this, std::placeholders::_1, 0.2, "right"));
  ClientNode::event_sub_ = create_subscription<cabot_msgs::msg::Log>("/cabot/activity_log",10,
      std::bind(&ClientNode::activity_log_callback, this, std::placeholders::_1));

  // starting thread
  std::thread(std::bind(&ClientNode::thread_diagnostics, this)).detach();
  std::thread(std::bind(&ClientNode::thread_plan, this)).detach();
}
*/
