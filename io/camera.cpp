#include "camera.hpp"

#include <stdexcept>

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
  auto exposure_ms = tools::read<double>(yaml, "exposure_ms");

  if (camera_name == "mindvision") {
    auto gamma = tools::read<double>(yaml, "gamma");
    auto vid_pid = yaml["vid_pid"] ? yaml["vid_pid"].as<std::string>() : std::string{};
    camera_ = std::make_unique<MindVision>(exposure_ms, gamma, vid_pid);
  }

  else if (camera_name == "hikrobot") {
    auto gain = yaml["gain"] ? yaml["gain"].as<double>() : 0.0;
    auto vid_pid = yaml["vid_pid"] ? yaml["vid_pid"].as<std::string>() : std::string{};
    auto serial_number =
      yaml["serial_number"] ? yaml["serial_number"].as<std::string>() : std::string{};
    camera_ = std::make_unique<HikRobot>(exposure_ms, gain, vid_pid, serial_number);
  }

  else {
    throw std::runtime_error("Unknow camera_name: " + camera_name + "!");
  }
}

void Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  camera_->read(img, timestamp);
  if (rotate_180_deg_ && !img.empty()) {
    cv::flip(img, img, -1);
  }
}

}  // namespace io