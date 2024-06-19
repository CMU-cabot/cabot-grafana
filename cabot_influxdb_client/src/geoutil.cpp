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
Point::Point(double x, double y)
: x(x), y(y) {}

std::string Point::toString() const
{
  char buffer[50];
  snprintf(buffer, sizeof(buffer), "(%.2f, %.2f)", x, y);
  return std::string(buffer);
}

// represent a global coordinate. init with lat, and lng
Latlng::Latlng(double lat, double lng)
: lat(lat), lng(lng) {}

std::string Latlng::toString() const
{
  char buffer[50];
  snprintf(buffer, sizeof(buffer), "(%.7f, %.7f)", lat, lng);
  return std::string(buffer);
}

// represent an anchor point. init with lat, lng, and rotate
Anchor::Anchor(double lat, double lng, double rotate)
: Latlng(lat, lng), rotate(rotate) {}

std::string Anchor::toString() const
{
  char buffer[70];
  snprintf(buffer, sizeof(buffer), "[%.7f, %.7f](%.2f)", lat, lng, rotate);
  return std::string(buffer);
}

// convert a LatLng point into a Mercator point
Point latlng2mercator(const Latlng & latlng)
{
  PJ * P = proj_create_crs_to_crs(PJ_DEFAULT_CTX, "EPSG:4326", "EPSG:3857", NULL);
  if (P == nullptr) {
    proj_destroy(P);
    throw std::runtime_error("Failed to create projection");
  }
  PJ * P_for_GIS = proj_normalize_for_visualization(PJ_DEFAULT_CTX, P);
  if (P_for_GIS == nullptr) {
    proj_destroy(P);
    proj_destroy(P_for_GIS);
    throw std::runtime_error("Failed to normalize projection");
  }
  proj_destroy(P);
  PJ_COORD a, b;
  a = proj_coord(latlng.lng, latlng.lat, 0, 0);
  b = proj_trans(P_for_GIS, PJ_FWD, a);
  proj_destroy(P_for_GIS);
  return Point(b.xy.x, b.xy.y);
}

// convert a Mercatro point into a LatLng point
Latlng mercator2latlng(const Point & mercator)
{
  PJ * P = proj_create_crs_to_crs(PJ_DEFAULT_CTX, "EPSG:3857", "EPSG:4326", NULL);
  if (P == nullptr) {
    proj_destroy(P);
    throw std::runtime_error("Failed to create projection");
  }
  PJ * P_for_GIS = proj_normalize_for_visualization(PJ_DEFAULT_CTX, P);
  if (P_for_GIS == nullptr) {
    proj_destroy(P);
    proj_destroy(P_for_GIS);
    throw std::runtime_error("Failed to normalize projection");
  }
  proj_destroy(P);
  PJ_COORD a, b;
  a = proj_coord(mercator.x, mercator.y, 0, 0);
  b = proj_trans(P_for_GIS, PJ_FWD, a);
  proj_destroy(P_for_GIS);
  return Latlng(b.xy.y, b.xy.x);
}

// get a resolution at an anchor point
double get_point_resolution(const Anchor & anchor)
{
  const double RADIUS = 6378137;
  return 1.0 / cosh(anchor.lat / RADIUS);
}

// convert a local point in the anchor coordinate into a Mercator point
Point xy2mercator(const Point & src_xy, const Anchor & anchor)
{
  Point mercator = latlng2mercator(anchor);
  Anchor temp;
  temp.lng = mercator.x;
  temp.lat = mercator.y;
  double r = get_point_resolution(temp);
  double x = src_xy.x;
  double y = src_xy.y;
  double rad = -anchor.rotate / 180.0 * M_PI;
  double c = cos(rad);
  double s = sin(rad);
  double dx = (x * c - y * s) / r;
  double dy = (x * s + y * c) / r;
  return Point(mercator.x + dx, mercator.y + dy);
}

// convert a local point in the anchor coordinate into the global point
Latlng local2global(const Point & xy, Anchor & anchor)
{
  Point mercator = xy2mercator(xy, anchor);
  return mercator2latlng(mercator);
}

// get anchor
Anchor get_anchor(const std::string & anchor_file)
{
  YAML::Node anchor = YAML::LoadFile(anchor_file);
  double lat = anchor["anchor"]["latitude"].as<double>();
  double lng = anchor["anchor"]["longitude"].as<double>();
  double rotate = anchor["anchor"]["rotate"].as<double>();
  return Anchor(lat, lng, rotate);
}
