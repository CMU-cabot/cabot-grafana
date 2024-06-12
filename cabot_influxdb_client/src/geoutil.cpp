/*******************************************************************************
 * Copyright (c) 2020, 2024  Carnegie Mellon University and Miraikan
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

/****************************************
 * Geography Utility
 *
 * Author: Daisuke Sato<daisukes@cmu.edu>
 ***************************************/

#include "geoutil.hpp"

// represent a 2D point
Point::Point(double x, double y) : x(x), y(y) {}
Point::Point(const geometry_msgs::msg::Point& point) : x(point.x), y(point.y) {}

// euclid distance between the instance point and the passed point
double Point::distance_to(const Point& other) const{
  return std::sqrt(std::pow(x - other.x, 2) + std::pow(y - other.y, 2));
}

// get a point between the instance point and the passed point defined by the 'ratio'. 'ratio'==1 means the passed point itself
Point Point::interpolate(const Point& other, double ratio) const{
  return Point(x * (1.0 - ratio) + other.x * ratio, y * (1.0 - ratio) + other.y * ratio);
}

// convert Point into ROS point msg
geometry_msgs::msg::Point Point::to_point_msg() const{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = 0.0;
  return point;
}

std::ostream& operator<<(std::ostream& os, const Point& point){
  os << "(" << point.x << ", " << point.y << ")";
  return os;
}

// represent a 2D pose. init with x, y, and r
Pose::Pose(double x, double y, double r) : Point(x, y), r(r) {}
Pose::Pose(const geometry_msgs::msg::Pose& pose) : Point(pose.position), r(tf2::getYaw(pose.orientation)) {}

geometry_msgs::msg::Quaternion Pose::get_orientation() const{
  tf2::Quaternion q;
  q.setRPY(0, 0, r);
  geometry_msgs::msg::Quaternion quat;
  tf2::convert(q, quat);
  return quat;
}

// convert Pose into ROS pose msg


// convert Pose into ROS pose msg
geometry_msgs::msg::Pose Pose::to_pose_msg() const{
  geometry_msgs::msg::Pose pose;
  pose.position = to_point_msg();
  pose.orientation = get_orientation();
  return pose;
}

// instanciate Pose from ROS pose msg"
Pose Pose::from_pose_msg(const geometry_msgs::msg::Pose& msg){
  tf2::Quaternion q;
  tf2::convert(msg.orientation, q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return Pose(msg.position.x, msg.position.y, yaw);
}

std::ostream& operator<<(std::ostream& os, const Pose& pose){
  os << "(" << pose.x << ", " << pose.y << ")[" << pose.r << " rad]";
  return os;
}

// represent a global coordinate. init with lat, and lng
Latlng::Latlng(double lat, double lng) : lat(lat), lng(lng) {}

// distance between the instance latlng and the passed latlng
double Latlng::distance_to(const Latlng& other) const{
  PJ_CONTEXT* C;
  PJ* P;
  PJ* P_for_GIS;

  C = proj_context_create();
  P = proj_create_crs_to_crs(C, "EPSG:4326", "EPSG:3857", NULL);
  if(0 == P){
    fprintf(stderr, "Oops\n");
    return -1;
  }

  P_for_GIS = proj_normalize_for_visualization(C, P);
  if(0 == P_for_GIS){
    fprintf(stderr, "Oops\n");
    return -1;
  }

  PJ_COORD a, b;

  a.lp.lam = proj_torad(lng);
  a.lp.phi = proj_torad(lat);
  b.lp.lam = proj_torad(other.lng);
  b.lp.phi = proj_torad(other.lat);

  a = proj_trans(P_for_GIS, PJ_FWD, a);
  b = proj_trans(P_for_GIS, PJ_FWD, b);

  proj_destroy(P);
  proj_context_destroy(C);

  return std::sqrt((a.xy.x - b.xy.x) * (a.xy.x - b.xy.x) + (a.xy.y - b.xy.y) * (a.xy.y - b.xy.y));
}

std::ostream& operator<<(std::ostream& os, const Latlng& latlng){
  os << "[" << latlng.lat << ", " << latlng.lng << "]";
  return os;
}

// represent an anchor point. init with lat, lng, and rotate
Anchor::Anchor(double lat, double lng, double rotate) : Latlng(lat, lng), rotate(rotate) {}

std::ostream& operator<<(std::ostream& os, const Anchor& anchor){
  os << "[" << anchor.lat << ", " << anchor.lng << "](" << anchor.rotate << " rad)";
  return os;
}

// this class should be used with Point
TargetPlace::TargetPlace(double x, double y, double r, double angle, int floor)
  : Pose(x, y, r), _angle(angle), _floor(floor), _was_approaching(false), _was_approached(false), _was_passed(false) {}

void TargetPlace::reset_target(){
  _was_approaching = false;
  _was_approached = false;
  _was_passed = false;
}

// the pose is approaching to the poi
bool TargetPlace::is_approaching(const Pose& pose){
  if(_was_approaching){
    return true;
  }
  if(distance_to(pose) < 5.0 && !_was_approaching){
    _was_approaching = true;
    _pose_approaching = pose;
    return true;
  }
  return false;
}

bool TargetPlace::is_approached(const Pose& pose){
  if(_was_approached){
    return true;
  }
  if(is_approaching(pose) && distance_to(pose) < 1.0 && !_was_approached){
    _was_approached = true;
    _pose_approached = pose;
    return true;
  }
  return false;
}

bool TargetPlace::is_passed(const Pose& pose){
  if(_was_passed){
    return true;
  }
  if(is_approached(pose) && distance_to(pose) > 1.0 && !_was_passed){
    _was_passed = true;
    _pose_passed = pose;
    return true;
  }
  return false;
}

bool TargetPlace::in_angle(const Pose& pose) const{
  double diff = std::fabs(r - pose.r);
  return diff <= _angle || diff >= (2 * M_PI - _angle);
}

// get Quaternion message from array
geometry_msgs::msg::Quaternion msg_from_q(const std::vector<double>& q){
  geometry_msgs::msg::Quaternion quat;
  quat.x = q[0];
  quat.y = q[1];
  quat.z = q[2];
  quat.w = q[3];
  return quat;
}

// get Point message from array
geometry_msgs::msg::Point msg_from_p(const std::vector<double>& p){
  geometry_msgs::msg::Point point;
  point.x = p[0];
  point.y = p[1];
  point.z = p[2];
  return point;
}

// get Pose message from pose and quaternion array
geometry_msgs::msg::Pose msg_from_pq(const std::vector<double>& p, const std::vector<double>& q){
  geometry_msgs::msg::Pose pose;
  pose.position = msg_from_p(p);
  pose.orientation = msg_from_q(q);
  return pose;
}

// get array from Quaternion message
std::vector<double> q_from_msg(const geometry_msgs::msg::Quaternion& msg){
  return {msg.x, msg.y, msg.z, msg.w};
}

// get array from Point message
std::vector<double> p_from_msg(const geometry_msgs::msg::Point& msg){
  return {msg.x, msg.y, msg.z};
}

// get quaternion array from two points
std::vector<double> q_from_points(const geometry_msgs::msg::Point& msg1, const geometry_msgs::msg::Point& msg2){
  tf2::Vector3 a(msg1.x, msg1.y, msg1.z);
  tf2::Vector3 b(msg2.x, msg2.y, msg2.z);
  tf2::Vector3 axis = a.cross(b);
  axis.normalize();
  double angle = std::acos(a.dot(b) / (a.length() * b.length()));
  tf2::Quaternion q(axis, angle);
  std::vector<double> quat(4);
  quat[0] = q.x();
  quat[1] = q.y();
  quat[2] = q.z();
  quat[3] = q.w();
  return quat;
}

// get inverse quaternion
std::vector<double> q_inverse(const std::vector<double>& q){
  return {-q[0], -q[1], -q[2], q[3]};
}

std::vector<double> q_diff(const std::vector<double>& q1, const std::vector<double>& q2){
  tf2::Quaternion a(q1[0], q1[1], q1[2], q1[3]);
  tf2::Quaternion b(q2[0], q2[1], q2[2], q2[3]);
  tf2::Quaternion diff = a.inverse() * b;
  std::vector<double> quat(4);
  quat[0] = diff.x();
  quat[1] = diff.y();
  quat[2] = diff.z();
  quat[3] = diff.w();
  return quat;
}

double get_yaw(const std::vector<double>& q){
  tf2::Quaternion quat(q[0], q[1], q[2], q[3]);
  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  return yaw;
}

// pose1 and pose2 is supporsed to be facing
bool in_angle(const geometry_msgs::msg::Pose& pose1, const geometry_msgs::msg::Pose& pose2, double margin_in_degree){
  /*
  pose1: robot pose
  pose2: POI pose
  margin_in_degree: POI angle margin in degree
  return True if robot orientation is in POI angle
  */
  double margin = margin_in_degree * M_PI / 180.0;
  // check orientation diff from pose2 orientation to rotated pose1 orientation
  // check orientation diff from pose2 orientation to rotated p1->p2 orientation
  double yaw1 = tf2::getYaw(pose1.orientation);
  double yaw2 = tf2::getYaw(pose2.orientation);
  double diff = std::fabs(yaw1 - yaw2);
  // check both in the margin
  return diff <= margin || diff >= (2 * M_PI - margin);
}

bool diff_in_angle(const std::vector<double>& quat1, const std::vector<double>& quat2, double margin_in_degree){
  /*
  margin_in_degree: angle margin in degree
  return True if two poses in margin
  */
  tf2::Quaternion q1(quat1[0], quat1[1], quat1[2], quat1[3]);
  tf2::Quaternion q2(quat2[0], quat2[1], quat2[2], quat2[3]);
  // check orientation diff from pose2 orientation to rotated pose1 orientation
  // check orientation diff from pose2 orientation to rotated p1->p2 orientation
  double yaw1 = tf2::getYaw(q1);
  double yaw2 = tf2::getYaw(q2);
  double margin = margin_in_degree * M_PI / 180.0;
  double diff = std::fabs(yaw1 - yaw2);
  // check both in the margin
  return diff <= margin || diff >= (2 * M_PI - margin);
}

double diff_angle(const geometry_msgs::msg::Quaternion& msg1, const geometry_msgs::msg::Quaternion& msg2){
  /*
  return yaw difference
  */
  tf2::Quaternion q1;
  tf2::Quaternion q2;
  tf2::convert(msg1, q1);
  tf2::convert(msg2, q2);
  double yaw1 = tf2::getYaw(q1);
  double yaw2 = tf2::getYaw(q2);
  return std::fabs(yaw1 - yaw2);
}

double get_rotation(const geometry_msgs::msg::Quaternion& src, const geometry_msgs::msg::Quaternion& target){
  tf2::Quaternion q_src;
  tf2::Quaternion q_target;
  tf2::convert(src, q_src);
  tf2::convert(target, q_target);
  tf2::Quaternion diff = q_src.inverse() * q_target;
  double roll, pitch, yaw;
  tf2::Matrix3x3(diff).getRPY(roll, pitch, yaw);
  return yaw;
}

Point get_projected_point_to_line(const Point& point, const Point& line_point, const geometry_msgs::msg::Quaternion& line_orientation){
  /*
  calculate the point that project a given point perpendicular to given line segment
  point: input point in numpy.array
  line_point: input line segment start point in numpy.array
  line_orientation: input line segment orientation in geometry_msgs.msg.Quaternion
  return point in numpy.array
  */
  double yaw = tf2::getYaw(line_orientation);
  double x_diff = point.x - line_point.x;
  double y_diff = point.y - line_point.y;
  double distance = x_diff * std::cos(yaw) + y_diff * std::sin(yaw);
  return Point(line_point.x + distance * std::cos(yaw), line_point.y + distance * std::sin(yaw));
}

bool is_forward_point(const geometry_msgs::msg::Pose& pose1, const geometry_msgs::msg::Point& pose2){
  /*
  calculate if the given point is in forward direction of the given pose
  pose1: robot pose
  pose2: POI pose
  return True if the point is in forward direction of the robot
  */
  double yaw = tf2::getYaw(pose1.orientation);
  double x_diff = pose2.x - pose1.position.x;
  double y_diff = pose2.y - pose1.position.y;
  double direction = std::atan2(y_diff, x_diff);
  double diff = std::fabs(yaw - direction);
  return diff < M_PI_2 || diff > 3 * M_PI_2;
}

// get anchor
Anchor get_anchor(const std::string& anchor_file){
  YAML::Node anchor = YAML::LoadFile(anchor_file);
  double lat = anchor["anchor"]["latitude"].as<double>();
  double lng = anchor["anchor"]["longitude"].as<double>();
  double rotate = anchor["anchor"]["rotate"].as<double>();
  // int floor = anchor["floor"].as<int>();
  return Anchor(lat, lng, rotate);
}

// convert a LatLng point into a Mercator point
Point latlng2mercator(const Latlng& latlng){
  if(latlng.lat < -85.05112878 || latlng.lat > 85.05112878){
    throw std::runtime_error("latitude over limit");
  }
  PJ_CONTEXT* C = proj_context_create();
  PJ* P = proj_create_crs_to_crs(C, "EPSG:4326", "EPSG:3857", NULL);
  if(0 == P){
    fprintf(stderr, "Oops\n");
    return Point();
  }
  PJ_COORD a, b;

  a.lp.lam = proj_torad(latlng.lng);
  a.lp.phi = proj_torad(latlng.lat);

  a = proj_trans(P, PJ_FWD, a);

  proj_destroy(P);
  proj_context_destroy(C);

  return Point(a.xy.x, a.xy.y);
}

// convert a Mercatro point into a LatLng point
Latlng mercator2latlng(const Point& mercator){
  PJ_CONTEXT* C = proj_context_create();
  PJ* P = proj_create_crs_to_crs(C, "EPSG:3857", "EPSG:4326", NULL);
  if(0 == P){
    fprintf(stderr, "Oops\n");
    return Latlng();
  }

  PJ_COORD a, b;

  a.xy.x = mercator.x;
  a.xy.y = mercator.y;

  a = proj_trans(P, PJ_INV, a);

  proj_destroy(P);
  proj_context_destroy(C);

  return Latlng(proj_todeg(a.lp.phi), proj_todeg(a.lp.lam));
}

// get a resolution at an anchor point
double get_point_resolution(const Anchor& anchor){
  PJ_CONTEXT* C = proj_context_create();
  PJ* P = proj_create_crs_to_crs(C, "EPSG:4326", "EPSG:3857", NULL);
  if(0 == P){
    fprintf(stderr, "Oops\n");
    return -1;
  }

  PJ_COORD a, b;

  a.lp.lam = proj_torad(anchor.lng);
  a.lp.phi = proj_torad(anchor.lat);

  a = proj_trans(P, PJ_FWD, a);

  proj_destroy(P);
  proj_context_destroy(C);

  return std::cos(proj_torad(anchor.lat)) * 2 * M_PI * 6378137 / 256;
}

// convert a Mercator point into a local point in the anchor coordinate
Point mercator2xy(const Point& src_mercator, const Anchor& anchor){
  Point anchor_mercator = latlng2mercator(anchor);
  double rotate = anchor.rotate;
  double dx = src_mercator.x - anchor_mercator.x;
  double dy = src_mercator.y - anchor_mercator.y;
  double cos_r = std::cos(rotate);
  double sin_r = std::sin(rotate);
  return Point(dx * cos_r - dy * sin_r, dx * sin_r + dy * cos_r);
}

// convert a local point in the anchor coordinate into a Mercator point
Point xy2mercator(const Point& src_xy, const Anchor& anchor){
  Point anchor_mercator = latlng2mercator(anchor);
  double rotate = anchor.rotate;
  double cos_r = std::cos(-rotate);
  double sin_r = std::sin(-rotate);
  double dx = src_xy.x * cos_r - src_xy.y * sin_r;
  double dy = src_xy.x * sin_r + src_xy.y * cos_r;
  return Point(dx + anchor_mercator.x, dy + anchor_mercator.y);
}

// convert a global point into a local point in the anchor coordinate
Point global2local(const Latlng& latlng, const Anchor& anchor){
  return mercator2xy(latlng2mercator(latlng), anchor);
}

// convert a local point in the anchor coordinate into the global point
Latlng local2global(const Point& xy, const Anchor& anchor){
  return mercator2latlng(xy2mercator(xy, anchor));
}
