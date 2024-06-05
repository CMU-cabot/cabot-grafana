#include "cabot_rclcpp_util.hpp"

std::shared_ptr<CaBotRclcppUtil> CaBotRclcppUtil::instance_ = nullptr;
std::mutex CaBotRclcppUtil::mutex_;

void CaBotRclcppUtil::initialize(std::shared_ptr<rclcpp::Node> node){
  if(!instance_){
    std::lock_guard<std::mutex> lock(mutex_);
    if(!instance_){
      if(!node){
        throw std::runtime_error("initialize should be colled with node fot the first time");
      }
      instance_ = std::shared_ptr<CaBotRclcppUtil>(new CaBotRclcppUtil(node));
    }
  }
}

std::shared_ptr<CaBotRclcppUtil> CaBotRclcppUtil::instance(){
  if(!instance_){
    throw  std::runtime_error("needs to be initialized first");
  }
  return instance_;
}

/*This utility class holds a node instance for convenience*/
CaBotRclcppUtil::CaBotRclcppUtil(std::shared_ptr<rclcpp::Node> node)
  : node_(node), logger_(node->get_logger()), clock_(node->get_clock()) {}

  void CaBotRclcppUtil::info(const std::string &msg) const{
    RCLCPP_INFO(logger_, msg.c_str());
  }

  void CaBotRclcppUtil::debug(const std::string &msg) const{
    RCLCPP_DEBUG(logger_, msg.c_str());
  }

  void CaBotRclcppUtil::error(const std::string &msg) const{
    RCLCPP_ERROR(logger_, msg.c_str());
  }

  void CaBotRclcppUtil::fatal(const std::string &msg) const{
    RCLCPP_FATAL(logger_, msg.c_str());
  }

  void CaBotRclcppUtil::warn(const std::string &msg) const{
    RCLCPP_WARN(logger_, msg.c_str());
  }

  rclcpp::Time CaBotRclcppUtil::now() const{
    return clock_->now();
  }

  double CaBotRclcppUtil::to_sec(const rclcpp::Duration &duration) const{
    return duration.nanoseconds() / 1e9;
  }

  rclcpp::Time CaBotRclcppUtil::time_zero() const{
    return rclcpp::Time(0, 0, clock_->get_clock_type());
  }
