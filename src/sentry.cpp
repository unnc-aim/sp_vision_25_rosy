#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <filesystem>
#include <memory>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "io/ros2/publish2nav.hpp"
#include "io/ros2/ros2.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/omniperception/decider.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }";

namespace
{
void draw_autoaim_overlay(
  cv::Mat & image, const std::list<auto_aim::Armor> & armors, const io::Command & command,
  const std::string & tracker_state)
{
  for (const auto & armor : armors) {
    tools::draw_points(image, armor.points, {0, 255, 0}, 2);
    auto label = fmt::format(
      "{}:{} {:.2f}", auto_aim::COLORS[armor.color], auto_aim::ARMOR_NAMES[armor.name],
      armor.confidence);
    tools::draw_text(image, label, armor.center, {0, 255, 0}, 0.6, 1);
  }

  auto status = fmt::format(
    "state={} ctrl={} shoot={} yaw={:.3f} pitch={:.3f}", tracker_state, command.control,
    command.shoot, command.yaw, command.pitch);
  tools::draw_text(image, status, {10, 30}, {0, 255, 255}, 0.7, 2);
}
}  // namespace

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);
  auto yaml = YAML::LoadFile(config_path);

  bool cboard_enabled = true;
  if (yaml["cboard_enabled"]) {
    cboard_enabled = yaml["cboard_enabled"].as<bool>();
  }

  bool use_usb_cameras = true;
  if (yaml["use_usb_cameras"]) {
    use_usb_cameras = yaml["use_usb_cameras"].as<bool>();
  }

  bool use_back_camera = true;
  if (yaml["use_back_camera"]) {
    use_back_camera = yaml["use_back_camera"].as<bool>();
  }

  auto config_dir = std::filesystem::path(config_path).parent_path();
  auto back_camera_config = (config_dir / "camera.yaml").string();

  io::ROS2 ros2;
  std::unique_ptr<io::CBoard> cboard;
  if (cboard_enabled) {
    cboard = std::make_unique<io::CBoard>(config_path);
  } else {
    tools::logger()->info(
      "[Sentry] CBoard disabled by config. Running with identity IMU and no CAN output.");
  }
  io::Camera camera(config_path);
  std::unique_ptr<io::Camera> back_camera;
  if (use_back_camera) {
    back_camera = std::make_unique<io::Camera>(back_camera_config);
  } else {
    tools::logger()->info(
      "[Sentry] Back camera disabled by config. Omniperception will reuse main camera.");
  }
  std::unique_ptr<io::USBCamera> usbcam1;
  std::unique_ptr<io::USBCamera> usbcam2;
  if (use_usb_cameras) {
    usbcam1 = std::make_unique<io::USBCamera>("video0", config_path);
    usbcam2 = std::make_unique<io::USBCamera>("video2", config_path);
  } else {
    tools::logger()->info(
      "[Sentry] USB cameras disabled by config. Omniperception will skip USB cameras.");
  }

  auto_aim::YOLO yolo(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  omniperception::Decider decider(config_path);

  cv::Mat img;

  std::chrono::steady_clock::time_point timestamp;
  io::Command last_command;

  while (!exiter.exit()) {
    camera.read(img, timestamp);
    if (img.empty()) {
      cv::Mat placeholder(720, 1280, CV_8UC3, cv::Scalar(0, 0, 0));
      tools::draw_text(placeholder, "SP Vision: no camera frame", {30, 60}, {0, 0, 255}, 1.0, 2);
      ros2.publish_raw_image(placeholder);
      ros2.publish_autoaim_image(placeholder);
      continue;
    }

    ros2.publish_raw_image(img);

    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    if (cboard) {
      q = cboard->imu_at(timestamp - 1ms);
    }
    // recorder.record(img, q, timestamp);

    /// 自瞄核心逻辑
    solver.set_R_gimbal2world(q);

    Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    auto armors = yolo.detect(img);

    decider.set_self_color(ros2.subscribe_self_color());

    decider.get_invincible_armor(ros2.subscribe_enemy_status());

    decider.armor_filter(armors);

    // decider.get_auto_aim_target(armors, ros2.subscribe_autoaim_target());

    decider.set_priority(armors);

    auto targets = tracker.track(armors, timestamp);

    io::Command command{false, false, 0, 0};

    /// 全向感知逻辑
    if (tracker.state() == "lost") {
      if (use_usb_cameras && usbcam1 && usbcam2) {
        command =
          decider.decide(yolo, gimbal_pos, *usbcam1, *usbcam2, back_camera ? *back_camera : camera);
      } else {
        command = decider.decide(yolo, gimbal_pos, back_camera ? *back_camera : camera);
      }
    } else
      command = aimer.aim(
        targets, timestamp, cboard ? cboard->bullet_speed : 0.0,
        cboard ? cboard->shoot_mode : io::ShootMode::left_shoot);

    /// 发射逻辑
    command.shoot = shooter.shoot(command, aimer, targets, gimbal_pos);

    if (cboard) {
      cboard->send(command);
    }
    ros2.publish_autoaim_command(command);

    cv::Mat autoaim_img = img.clone();
    draw_autoaim_overlay(autoaim_img, armors, command, tracker.state());
    ros2.publish_autoaim_image(autoaim_img);

    /// ROS2通信
    Eigen::Vector4d target_info = decider.get_target_info(armors, targets);

    ros2.publish(target_info);
  }
  return 0;
}