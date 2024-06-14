/*******************************************************************************
 * Copyright (c) 2022, 2024  Carnegie Mellon University and Miraikan
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

#ifndef CABOT_RCLCPP_UTIL_HPP_
#define CABOT_RCLCPP_UTIL_HPP_

#include <memory>
#include <mutex>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>

class CaBotRclcppUtil {
  public:
    static void initialize(std::shared_ptr<rclcpp::Node> node);
    static std::shared_ptr<CaBotRclcppUtil> instance();

    void info(const std::string &msg) const;
    void debug(const std::string &msg) const;
    void error(const std::string &msg) const;
    void fatal(const std::string &msg) const;
    void warn(const std::string &msg) const;

    rclcpp::Time now() const;
    double to_sec(const rclcpp::Duration &duration) const;
    rclcpp::Time time_zero() const;

  private:
    CaBotRclcppUtil(std::shared_ptr<rclcpp::Node> node);
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Logger logger_;
    rclcpp::Clock::SharedPtr clock_;

    static std::shared_ptr<CaBotRclcppUtil> instance_;
    static std::mutex mutex_;
};

#endif  // CABOT_RCLCPP_UTIL_HPP_
