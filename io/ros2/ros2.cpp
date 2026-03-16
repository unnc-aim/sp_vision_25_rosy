#include "ros2.hpp"
namespace io
{
ROS2::ROS2()
{
  rclcpp::init(0, nullptr);

  publish2nav_ = std::make_shared<Publish2Nav>();

  subscribe2nav_ = std::make_shared<Subscribe2Nav>();

  publish_spin_thread_ = std::make_unique<std::thread>([this]() { publish2nav_->start(); });

  subscribe_spin_thread_ = std::make_unique<std::thread>([this]() { subscribe2nav_->start(); });
}

ROS2::~ROS2()
{
  rclcpp::shutdown();
  publish_spin_thread_->join();
  subscribe_spin_thread_->join();
}

void ROS2::publish(const Eigen::Vector4d & target_pos) { publish2nav_->send_data(target_pos); }

void ROS2::publish_autoaim_command(const io::Command & command)
{
  publish2nav_->send_autoaim_command(command);
}

void ROS2::publish_raw_image(const cv::Mat & image) { publish2nav_->send_raw_image(image); }

void ROS2::publish_autoaim_image(const cv::Mat & image) { publish2nav_->send_autoaim_image(image); }

std::vector<int8_t> ROS2::subscribe_enemy_status()
{
  return subscribe2nav_->subscribe_enemy_status();
}

std::vector<int8_t> ROS2::subscribe_autoaim_target()
{
  return subscribe2nav_->subscribe_autoaim_target();
}

std::string ROS2::subscribe_self_color() { return subscribe2nav_->subscribe_self_color(); }

Eigen::Quaterniond ROS2::subscribe_imu() { return subscribe2nav_->subscribe_imu(); }

}  // namespace io
