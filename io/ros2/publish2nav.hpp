#ifndef IO__PBLISH2NAV_HPP
#define IO__PBLISH2NAV_HPP

#include <Eigen/Dense>  // For Eigen::Vector3d
#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "io/command.hpp"
#include "opencv2/core/mat.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sp_msgs/msg/auto_aim_command_msg.hpp"
#include "std_msgs/msg/string.hpp"

namespace io
{
class Publish2Nav : public rclcpp::Node
{
public:
  Publish2Nav();

  ~Publish2Nav();

  void start();

  void send_data(const Eigen::Vector4d & data);

  void send_autoaim_command(const io::Command & command);

  void send_raw_image(const cv::Mat & image);

  void send_autoaim_image(const cv::Mat & image);

private:
  sensor_msgs::msg::Image cv_to_image_msg(
    const cv::Mat & image, const std::string & frame_id, const rclcpp::Time & stamp) const;

  // ROS2 发布者
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Publisher<sp_msgs::msg::AutoAimCommandMsg>::SharedPtr autoaim_command_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr autoaim_image_publisher_;
};

}  // namespace io

#endif  // Publish2Nav_HPP_
