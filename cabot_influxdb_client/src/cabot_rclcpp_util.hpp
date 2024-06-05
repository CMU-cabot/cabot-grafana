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
