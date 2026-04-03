#include "subscribe2nav.hpp"

#include <sstream>
#include <vector>

namespace io
{

Subscribe2Nav::Subscribe2Nav()
: Node("nav_subscriber"),
  enemy_statue_queue_(1),
  autoaim_target_queue_(1),
  imu_queue_(1),
  self_color_(dji_referee_protocol::msg::Constants::COLOR_UNKNOWN),
  enemy_status_counter_(0),
  autoaim_target_counter_(0)
{
  enemy_status_subscription_ = this->create_subscription<sp_msgs::msg::EnemyStatusMsg>(
    "enemy_status", 10,
    std::bind(&Subscribe2Nav::enemy_status_callback, this, std::placeholders::_1));

  autoaim_target_subscription_ = this->create_subscription<sp_msgs::msg::AutoaimTargetMsg>(
    "autoaim_target", 10,
    std::bind(&Subscribe2Nav::autoaim_target_callback, this, std::placeholders::_1));

  self_color_subscription_ = this->create_subscription<dji_referee_protocol::msg::SelfColor>(
    "/referee/parsed/common/self_color", 10,
    std::bind(&Subscribe2Nav::self_color_callback, this, std::placeholders::_1));

  // Subscribe to IMU data from EtherCAT controller (BEST_EFFORT to match publisher QoS)
  this->declare_parameter("imu_topic", "/ecat/sn4653115/app2/read");
  std::string imu_topic = this->get_parameter("imu_topic").as_string();

  rclcpp::QoS imu_qos(1);
  imu_qos.best_effort();
  imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, imu_qos,
    std::bind(&Subscribe2Nav::imu_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "nav_subscriber node initialized with IMU subscription on topic: %s", imu_topic.c_str());
}

void Subscribe2Nav::self_color_callback(const dji_referee_protocol::msg::SelfColor::SharedPtr msg)
{
  self_color_ = msg->color;
}

Subscribe2Nav::~Subscribe2Nav()
{
  RCLCPP_INFO(this->get_logger(), "nav_subscriber node shutting down.");
}

void Subscribe2Nav::enemy_status_callback(const sp_msgs::msg::EnemyStatusMsg::SharedPtr msg)
{
  enemy_statue_queue_.clear();
  enemy_statue_queue_.push(*msg);

  enemy_status_counter_++;

  if (enemy_status_counter_ >= 2) {
    if (enemy_status_timer_) {
      enemy_status_timer_->cancel();
    }
    enemy_status_timer_ = this->create_wall_timer(std::chrono::milliseconds(1500), [this]() {
      enemy_statue_queue_.clear();
      enemy_status_counter_ = 0;
      RCLCPP_INFO(
        this->get_logger(), "Enemy status queue cleared due to inactivity after two messages.");
    });
  }
}

void Subscribe2Nav::autoaim_target_callback(const sp_msgs::msg::AutoaimTargetMsg::SharedPtr msg)
{
  autoaim_target_queue_.clear();
  autoaim_target_queue_.push(*msg);

  autoaim_target_counter_++;

  if (autoaim_target_counter_ >= 2) {
    if (autoaim_target_timer_) {
      autoaim_target_timer_->cancel();
    }
    autoaim_target_timer_ = this->create_wall_timer(std::chrono::milliseconds(1500), [this]() {
      autoaim_target_queue_.clear();
      autoaim_target_counter_ = 0;
      RCLCPP_INFO(
        this->get_logger(), "Autoaim target queue cleared due to inactivity after two messages.");
    });
  }
}

void Subscribe2Nav::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  imu_queue_.clear();
  imu_queue_.push(*msg);
}

void Subscribe2Nav::start()
{
  RCLCPP_INFO(this->get_logger(), "nav_subscriber node Starting to spin...");
  rclcpp::spin(this->shared_from_this());
}

std::vector<int8_t> Subscribe2Nav::subscribe_enemy_status()
{
  if (enemy_statue_queue_.empty()) {
    return std::vector<int8_t>();
  }
  sp_msgs::msg::EnemyStatusMsg msg;

  enemy_statue_queue_.back(msg);
  RCLCPP_INFO(
    this->get_logger(), "Subscribe enemy_status at: %d.%09u", msg.timestamp.sec,
    msg.timestamp.nanosec);

  return msg.invincible_enemy_ids;
}

std::vector<int8_t> Subscribe2Nav::subscribe_autoaim_target()
{
  if (autoaim_target_queue_.empty()) {
    return std::vector<int8_t>();
  }
  sp_msgs::msg::AutoaimTargetMsg msg;

  autoaim_target_queue_.back(msg);
  RCLCPP_INFO(
    this->get_logger(), "Subscribe autoaim_target at: %d.%09u", msg.timestamp.sec,
    msg.timestamp.nanosec);

  return msg.target_ids;
}

uint8_t Subscribe2Nav::subscribe_self_color() { return self_color_; }

Eigen::Quaterniond Subscribe2Nav::subscribe_imu()
{
  if (imu_queue_.empty()) {
    return Eigen::Quaterniond::Identity();
  }
  sensor_msgs::msg::Imu msg;
  imu_queue_.back(msg);

  Eigen::Quaterniond q;
  q.w() = msg.orientation.w;
  q.x() = msg.orientation.x;
  q.y() = msg.orientation.y;
  q.z() = msg.orientation.z;

  return q;
}

}  // namespace io
