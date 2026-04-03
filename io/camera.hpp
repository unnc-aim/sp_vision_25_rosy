#ifndef IO__CAMERA_HPP
#define IO__CAMERA_HPP

#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

#if __has_include(<rclcpp/rclcpp.hpp>)
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#define SP_VISION_HAS_RCLCPP 1
#endif

namespace io
{
class CameraBase
{
public:
  virtual ~CameraBase() = default;
  virtual void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) = 0;
  virtual void set_exposure_ms(double exposure_ms) { (void)exposure_ms; }
  virtual void set_gain(double gain) { (void)gain; }
};

class Camera
{
public:
  Camera(const std::string & config_path);
  ~Camera();
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp);

private:
  std::unique_ptr<CameraBase> camera_;
  bool rotate_180_deg_;
  double exposure_ms_;
  double gain_;

#ifdef SP_VISION_HAS_RCLCPP
  std::shared_ptr<rclcpp::Node> param_node_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> param_executor_;
  std::thread param_spin_thread_;
#endif
};

}  // namespace io

#endif  // IO__CAMERA_HPP