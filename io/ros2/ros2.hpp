#ifndef IO__ROS2_HPP
#define IO__ROS2_HPP

#include <opencv2/core/mat.hpp>

#include "io/command.hpp"
#include "publish2nav.hpp"
#include "subscribe2nav.hpp"

namespace io
{
class ROS2
{
public:
  ROS2();

  ~ROS2();

  void publish(const Eigen::Vector4d & target_pos);

  void publish_autoaim_command(const io::Command & command);

  void publish_raw_image(const cv::Mat & image);

  void publish_autoaim_image(const cv::Mat & image);

  std::vector<int8_t> subscribe_enemy_status();

  std::vector<int8_t> subscribe_autoaim_target();

  std::string subscribe_self_color();

  template <typename T>
  std::shared_ptr<rclcpp::Publisher<T>> create_publisher(
    const std::string & node_name, const std::string & topic_name, size_t queue_size)
  {
    auto node = std::make_shared<rclcpp::Node>(node_name);

    auto publisher = node->create_publisher<T>(topic_name, queue_size);

    // 运行一个单独的线程来 spin 这个节点，确保消息可以被正确发布
    std::thread([node]() { rclcpp::spin(node); }).detach();

    return publisher;
  }

private:
  std::shared_ptr<Publish2Nav> publish2nav_;
  std::shared_ptr<Subscribe2Nav> subscribe2nav_;

  std::unique_ptr<std::thread> publish_spin_thread_;
  std::unique_ptr<std::thread> subscribe_spin_thread_;
};

}  // namespace io
#endif