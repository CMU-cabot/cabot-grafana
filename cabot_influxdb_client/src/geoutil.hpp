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

#ifndef GEOUTIL_HPP_
#define GEOUTIL_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <vector>
#include <string>
#include <iostream>

#include <cassert>
#include <iomanip>
#include <proj/coordinateoperation.hpp>
#include <proj/crs.hpp>
#include <proj/io.hpp>
#include <proj/util.hpp>

#include "cabot_rclcpp_util.hpp"

class Point{
public:
  double x, y;
  Point(double x = 0.0, double y = 0.0);
  Point(const geometry_msgs::msg::Point& point);
  double distance_to(const Point& other) const;
  Point interpolate(const Point& other, double ratio) const;
  geometry_msgs::msg::Point to_point_msg() const;
  friend std::ostream& operator<<(std::ostream& os, const Point& point);
};

class Pose : public Point{
public:
  double r;
  Pose(double x = 0.0, double y = 0.0, double r = 0.0);
  Pose(const geometry_msgs::msg::Pose& pose);
  //pose_from_point
  geometry_msgs::msg::Quaternion get_orientation() const;
  static Pose from_pose_msg(const geometry_msgs::msg::Pose& msg);
  geometry_msgs::msg::Pose to_pose_msg() const;
  //to_pose_stamped_msg
  friend std::ostream& operator<<(std::ostream& os, const Pose& pose);
};

class Latlng{
public:
  double lat, lng;
  Latlng(double lat = 0.0, double lng = 0.0);
  double distance_to(const Latlng& other) const;
  friend std::ostream& operator<<(std::ostream& os, const Latlng& latlng);
};


class Anchor : public Latlng{
public:
  double rotate;
  Anchor(double lat = 0.0, double lng = 0.0, double rotate = 0.0);
  friend std::ostream& operator<<(std::ostream& os, const Anchor& anchor);
};

class TargetPlace : public Pose{
public:
  double _angle;
  int _floor;
  TargetPlace(double x = 0.0, double y = 0.0, double r = 0.0, double angle = 0.0, int floor = 0);
  void reset_target();
  bool in_angle(const Pose& pose) const;
  bool is_approaching(const Pose& pose);
  bool is_approached(const Pose& pose);
  bool is_passed(const Pose& pose);

private:
  bool _was_approaching;
  Pose _pose_approaching;
  bool _was_approached;
  Pose _pose_approached;
  bool _was_passed;
  Pose _pose_passed;
};
 
geometry_msgs::msg::Quaternion msg_from_q(const std::vector<double>& q);
geometry_msgs::msg::Point msg_from_p(const std::vector<double>& p);
geometry_msgs::msg::Pose msg_from_pq(const std::vector<double>& p, const std::vector<double>& q);
std::vector<double> q_from_msg(const geometry_msgs::msg::Quaternion& msg);
std::vector<double> p_from_msg(const geometry_msgs::msg::Point& msg);
std::vector<double> q_from_points(const geometry_msgs::msg::Point& msg1, const geometry_msgs::msg::Point& msg2);
std::vector<double> q_inverse(const std::vector<double>& q);
std::vector<double> q_diff(const std::vector<double>& q1, const std::vector<double>& q2);
double get_yaw(const std::vector<double>& q);
bool in_angle(const geometry_msgs::msg::Pose& pose1, const geometry_msgs::msg::Pose& pose2, double margin_in_degree);
bool diff_in_angle(const std::vector<double>& quat1, const std::vector<double>& quat2, double margin_in_degree);
double diff_angle(const geometry_msgs::msg::Quaternion& msg1, const geometry_msgs::msg::Quaternion& msg2);
double get_rotation(const geometry_msgs::msg::Quaternion& src, const geometry_msgs::msg::Quaternion& target);
Point get_projected_point_to_line(const Point& point, const Point& line_point, const geometry_msgs::msg::Quaternion& line_orientation);
bool is_forward_point(const Point& pose1, const geometry_msgs::msg::Point& pose2);

Point latlng2mercator(const std::string& latlng);
Latlng mercator2latlng(const Point& mercator);
double get_point_resolution(const Anchor& anchor);
Point mercator2xy(const Point& src_mercator, const Anchor& anchor);
Point xy2mercator(const Point& src_xy, const Anchor& anchor);
Point global2local(const Latlng& latlng, const Anchor& anchor);
Latlng local2global(const Point& xy, const Anchor& anchor);
Anchor get_anchor(const std::string& anchor_file);

#endif  // GEOUTIL_HPP_
