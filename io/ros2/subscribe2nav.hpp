#ifndef IO__SUBSCRIBE2NAV_HPP
#define IO__SUBSCRIBE2NAV_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <sp_msgs/msg/detail/autoaim_target_msg__struct.hpp>
#include <string>
#include <vector>
#include <sensor_msgs/msg/imu.hpp>
#include <Eigen/Geometry>

#include "sp_msgs/msg/autoaim_target_msg.hpp"
#include "sp_msgs/msg/enemy_status_msg.hpp"
#include "std_msgs/msg/string.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{
class Subscribe2Nav : public rclcpp::Node
{
public:
  Subscribe2Nav();

  ~Subscribe2Nav();

  void start();

  std::vector<int8_t> subscribe_enemy_status();
  std::vector<int8_t> subscribe_autoaim_target();
  std::string subscribe_self_color();
  Eigen::Quaterniond subscribe_imu();

private:
  void enemy_status_callback(const sp_msgs::msg::EnemyStatusMsg::SharedPtr msg);
  void autoaim_target_callback(const sp_msgs::msg::AutoaimTargetMsg::SharedPtr msg);
  void self_color_callback(const std_msgs::msg::String::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

  int enemy_status_counter_;
  int autoaim_target_counter_;

  rclcpp::TimerBase::SharedPtr enemy_status_timer_;
  rclcpp::TimerBase::SharedPtr autoaim_target_timer_;

  rclcpp::Subscription<sp_msgs::msg::EnemyStatusMsg>::SharedPtr enemy_status_subscription_;
  rclcpp::Subscription<sp_msgs::msg::AutoaimTargetMsg>::SharedPtr autoaim_target_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr self_color_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;

  tools::ThreadSafeQueue<sp_msgs::msg::EnemyStatusMsg> enemy_statue_queue_;
  tools::ThreadSafeQueue<sp_msgs::msg::AutoaimTargetMsg> autoaim_target_queue_;
  tools::ThreadSafeQueue<sensor_msgs::msg::Imu> imu_queue_;
  std::string self_color_;
};
}  // namespace io

#endif  // IO__SUBSCRIBE2NAV_HPP
