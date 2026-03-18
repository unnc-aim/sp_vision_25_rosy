#include "publish2nav.hpp"

#include <Eigen/Dense>
#include <chrono>
#include <cstring>
#include <memory>
#include <thread>

#include "tools/logger.hpp"

namespace io
{

Publish2Nav::Publish2Nav() : Node("auto_aim_target_pos_publisher")
{
  publisher_ = this->create_publisher<std_msgs::msg::String>("auto_aim_target_pos", 10);
  autoaim_command_publisher_ =
    this->create_publisher<sp_msgs::msg::AutoAimCommandMsg>("/sp_vision/autoaim_command", 10);
  raw_image_publisher_ =
    this->create_publisher<sensor_msgs::msg::Image>("/sp_vision/image_raw", 10);
  autoaim_image_publisher_ =
    this->create_publisher<sensor_msgs::msg::Image>("/sp_vision/image_autoaim", 10);
  profile_log_publisher_ =
    this->create_publisher<std_msgs::msg::String>("/sp_vision/profile_log", 100);

  RCLCPP_INFO(this->get_logger(), "auto_aim_target_pos_publisher node initialized.");
}

void Publish2Nav::send_autoaim_command(const io::Command & command)
{
  sp_msgs::msg::AutoAimCommandMsg message;
  message.timestamp = this->now();
  message.control = command.control;
  message.shoot = command.shoot;
  message.yaw = command.yaw;
  message.pitch = command.pitch;
  autoaim_command_publisher_->publish(message);
}

sensor_msgs::msg::Image Publish2Nav::cv_to_image_msg(
  const cv::Mat & image, const std::string & frame_id, const rclcpp::Time & stamp) const
{
  sensor_msgs::msg::Image msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.height = static_cast<uint32_t>(image.rows);
  msg.width = static_cast<uint32_t>(image.cols);
  msg.encoding = "bgr8";
  msg.is_bigendian = false;
  msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(image.cols * image.elemSize());
  msg.data.resize(msg.step * msg.height);

  if (image.isContinuous()) {
    std::memcpy(msg.data.data(), image.data, msg.data.size());
  } else {
    for (int row = 0; row < image.rows; ++row) {
      std::memcpy(msg.data.data() + row * msg.step, image.ptr(row), msg.step);
    }
  }

  return msg;
}

void Publish2Nav::send_raw_image(const cv::Mat & image)
{
  if (image.empty()) return;
  raw_image_publisher_->publish(cv_to_image_msg(image, "sp_vision_raw", this->now()));
}

void Publish2Nav::send_autoaim_image(const cv::Mat & image)
{
  if (image.empty()) return;
  autoaim_image_publisher_->publish(cv_to_image_msg(image, "sp_vision_autoaim", this->now()));
}

void Publish2Nav::send_profile_log(const std::string & line)
{
  std_msgs::msg::String msg;
  msg.data = line;
  profile_log_publisher_->publish(msg);
}

Publish2Nav::~Publish2Nav()
{
  RCLCPP_INFO(this->get_logger(), "auto_aim_target_pos_publisher node shutting down.");
}

void Publish2Nav::send_data(const Eigen::Vector4d & target_pos)
{
  // 创建消息
  auto message = std::make_shared<std_msgs::msg::String>();

  // 将 Eigen::Vector3d 数据转换为字符串并存储在消息中
  message->data = std::to_string(target_pos[0]) + "," + std::to_string(target_pos[1]) + "," +
                  std::to_string(target_pos[2]) + "," + std::to_string(target_pos[3]);

  // 发布消息
  publisher_->publish(*message);

  // RCLCPP_INFO(
  //   this->get_logger(), "auto_aim_target_pos_publisher node sent message: '%s'",
  //   message->data.c_str());
}

void Publish2Nav::start()
{
  RCLCPP_INFO(this->get_logger(), "auto_aim_target_pos_publisher node starting to spin...");
  rclcpp::spin(this->shared_from_this());
}

}  // namespace io
