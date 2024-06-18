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
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <vector>
#include <string>
#include <cstdio>
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
  Point(double x, double y);
  friend std::ostream& operator<<(std::ostream& os, const Point& point);
  std::string toString() const;
};

class Latlng{
public:
  double lat, lng;
  Latlng(double lat, double lng);
  friend std::ostream& operator<<(std::ostream& os, const Latlng& latlng);
  std::string toString() const;
};

class Anchor : public Latlng{
public:
  double rotate = 0.0 ;
  Anchor(double lat = 0.0, double lng = 0.0, double rotate = 0.0);
  friend std::ostream& operator<<(std::ostream& os, const Anchor& anchor);
  std::string toString() const;
};

Point latlng2mercator(const Latlng& latlng);
Latlng mercator2latlng(const Point& mercator);
double get_point_resolution(const Anchor& anchor);
Point xy2mercator(const Point& src_xy, const Anchor& anchor);
Latlng local2global(const Point& xy, Anchor& anchor);
Anchor get_anchor(const std::string& anchor_file);

#endif  // GEOUTIL_HPP_
