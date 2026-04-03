#include "camera.hpp"

#include <stdexcept>
#include <utility>

#include "hikrobot/hikrobot.hpp"
#include "mindvision/mindvision.hpp"
#include "tools/yaml.hpp"

namespace io
{
Camera::Camera(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  rotate_180_deg_ = yaml["rotate_180_deg"] ? yaml["rotate_180_deg"].as<bool>() : false;
  auto camera_name = tools::read<std::string>(yaml, "camera_name");
  exposure_ms_ = tools::read<double>(yaml, "exposure_ms");
  gain_ = yaml["gain"] ? yaml["gain"].as<double>() : 0.0;

  if (camera_name == "mindvision") {
    auto gamma = tools::read<double>(yaml, "gamma");
    auto vid_pid = yaml["vid_pid"] ? yaml["vid_pid"].as<std::string>() : std::string{};
    camera_ = std::make_unique<MindVision>(exposure_ms_, gamma, vid_pid);
  }

  else if (camera_name == "hikrobot") {
    auto vid_pid = yaml["vid_pid"] ? yaml["vid_pid"].as<std::string>() : std::string{};
    auto serial_number =
      yaml["serial_number"] ? yaml["serial_number"].as<std::string>() : std::string{};
    camera_ = std::make_unique<HikRobot>(exposure_ms_, gain_, vid_pid, serial_number);
  }

  else {
    throw std::runtime_error("Unknow camera_name: " + camera_name + "!");
  }

#ifdef SP_VISION_HAS_RCLCPP
  if (rclcpp::ok()) {
    static std::atomic<uint32_t> node_id{0};
    auto id = node_id.fetch_add(1, std::memory_order_relaxed);
    param_node_ = std::make_shared<rclcpp::Node>("sp_vision_camera_params_" + std::to_string(id));

    param_node_->declare_parameter("exposure_ms", exposure_ms_);
    param_node_->declare_parameter("gain", gain_);

    param_callback_handle_ = param_node_->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        for (const auto & param : params) {
          if (param.get_name() == "exposure_ms") {
            if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
              result.successful = false;
              result.reason = "exposure_ms must be double";
              return result;
            }
            exposure_ms_ = param.as_double();
            camera_->set_exposure_ms(exposure_ms_);
          } else if (param.get_name() == "gain") {
            if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
              result.successful = false;
              result.reason = "gain must be double";
              return result;
            }
            gain_ = param.as_double();
            camera_->set_gain(gain_);
          }
        }

        return result;
      });

    param_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    param_executor_->add_node(param_node_);
    param_spin_thread_ = std::thread([this] { param_executor_->spin(); });
  }
#endif
}

Camera::~Camera()
{
#ifdef SP_VISION_HAS_RCLCPP
  if (param_executor_) {
    param_executor_->cancel();
  }
  if (param_spin_thread_.joinable()) {
    param_spin_thread_.join();
  }
  if (param_executor_ && param_node_) {
    param_executor_->remove_node(param_node_);
  }
#endif
}

void Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  camera_->read(img, timestamp);
  if (rotate_180_deg_ && !img.empty()) {
    cv::flip(img, img, -1);
  }
}

}  // namespace io