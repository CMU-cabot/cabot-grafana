//
// Copyright (c) 2022, 2024  Carnegie Mellon University and Miraikan
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "cabot_rclcpp_util.hpp"

std::shared_ptr<CaBotRclcppUtil> CaBotRclcppUtil::instance_ = nullptr;
std::mutex CaBotRclcppUtil::mutex_;

// This utility class holds a node instance for convenience
void CaBotRclcppUtil::initialize(std::shared_ptr<rclcpp::Node> node)
{
  if (!instance_) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!instance_) {
      if (!node) {
        throw std::runtime_error("initialize should be colled with node fot the first time");
      }
      instance_ = std::shared_ptr<CaBotRclcppUtil>(new CaBotRclcppUtil(node));
    }
  }
}

std::shared_ptr<CaBotRclcppUtil> CaBotRclcppUtil::instance()
{
  if (!instance_) {
    throw  std::runtime_error("needs to be initialized first");
  }
  return instance_;
}

// initialize
CaBotRclcppUtil::CaBotRclcppUtil(std::shared_ptr<rclcpp::Node> node)
: node_(node), logger_(node->get_logger()), clock_(node->get_clock()) {}

void CaBotRclcppUtil::info(const std::string & msg) const
{
  RCLCPP_INFO(logger_, msg.c_str());
}

void CaBotRclcppUtil::debug(const std::string & msg) const
{
  RCLCPP_DEBUG(logger_, msg.c_str());
}

void CaBotRclcppUtil::error(const std::string & msg) const
{
  RCLCPP_ERROR(logger_, msg.c_str());
}

void CaBotRclcppUtil::fatal(const std::string & msg) const
{
  RCLCPP_FATAL(logger_, msg.c_str());
}

void CaBotRclcppUtil::warn(const std::string & msg) const
{
  RCLCPP_WARN(logger_, msg.c_str());
}

rclcpp::Time CaBotRclcppUtil::now() const
{
  return clock_->now();
}

double CaBotRclcppUtil::to_sec(const rclcpp::Duration & duration) const
{
  return duration.nanoseconds() / 1e9;
}

rclcpp::Time CaBotRclcppUtil::time_zero() const
{
  return rclcpp::Time(0, 0, clock_->get_clock_type());
}
