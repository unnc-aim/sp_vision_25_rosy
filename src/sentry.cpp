#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <chrono>
#include <filesystem>
#include <memory>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
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
  const std::string & tracker_state, const std::string & search_color,
  const std::string & self_color, double fps, const std::optional<cv::Point> & aim_point_px,
  const std::optional<double> & final_x)
{
  for (const auto & armor : armors) {
    if (armor.box.width > 0 && armor.box.height > 0) {
      cv::rectangle(image, armor.box, cv::Scalar(0, 255, 0), 2);
    }

    if (armor.points.size() >= 4) {
      std::array<cv::Point, 4> pts = {
        cv::Point(static_cast<int>(armor.points[0].x), static_cast<int>(armor.points[0].y)),
        cv::Point(static_cast<int>(armor.points[1].x), static_cast<int>(armor.points[1].y)),
        cv::Point(static_cast<int>(armor.points[2].x), static_cast<int>(armor.points[2].y)),
        cv::Point(static_cast<int>(armor.points[3].x), static_cast<int>(armor.points[3].y))};

      // 装甲板轮廓
      for (int i = 0; i < 4; ++i) {
        cv::line(image, pts[i], pts[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
      }

      // 左右灯条（左:0-3, 右:1-2）
      cv::line(image, pts[0], pts[3], cv::Scalar(255, 0, 0), 2);
      cv::line(image, pts[1], pts[2], cv::Scalar(255, 0, 0), 2);
    }

    tools::draw_points(image, armor.points, {0, 255, 0}, 2);
    cv::circle(image, armor.center, 3, cv::Scalar(0, 255, 255), -1);
    auto label = fmt::format(
      "{}:{} {:.2f}", auto_aim::COLORS[armor.color], auto_aim::ARMOR_NAMES[armor.name],
      armor.confidence);
    cv::Point text_pos{armor.box.x, std::max(20, armor.box.y - 8)};
    tools::draw_text(image, label, text_pos, {0, 255, 0}, 0.6, 2);
  }

  auto color_status = fmt::format("self_color={} search_color={}", self_color, search_color);
  tools::draw_text(image, color_status, {10, 30}, {255, 255, 0}, 0.8, 2);

  auto status = fmt::format(
    "state={} ctrl={} shoot={} yaw={:.3f} pitch={:.3f}", tracker_state, command.control,
    command.shoot, command.yaw, command.pitch);
  tools::draw_text(image, status, {10, 60}, {0, 255, 255}, 0.7, 2);

  auto fps_text = fmt::format("fps={:.1f}", fps);
  tools::draw_text(image, fps_text, {10, 90}, {255, 255, 255}, 0.7, 2);

  // if (final_x.has_value()) {
  //   auto final_x_text = fmt::format("final_x={:.3f}", final_x.value());
  //   tools::draw_text(image, final_x_text, {10, 120}, {255, 255, 255}, 0.7, 2);
  // }

  if (aim_point_px.has_value()) {
    auto p = aim_point_px.value();
    cv::drawMarker(image, p, cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 20, 2);
    tools::draw_text(image, "aim_point", {p.x + 8, p.y - 8}, {0, 0, 255}, 0.6, 2);
  }
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
  auto fps_window_begin = std::chrono::steady_clock::now();
  int fps_window_frames = 0;
  double publish_fps = 0.0;
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
    } else {
      q = ros2.subscribe_imu();
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
        command = decider.decide(
          yolo, gimbal_pos, *usbcam1, *usbcam2, back_camera ? *back_camera : camera,
          use_back_camera);
      } else {
        command =
          decider.decide(yolo, gimbal_pos, back_camera ? *back_camera : camera, use_back_camera);
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

    auto search_color = decider.current_search_color_text();
    auto self_color = decider.current_self_color_text();

    // 统计自瞄图输出帧率（1秒窗口）
    fps_window_frames++;
    auto now = std::chrono::steady_clock::now();
    auto fps_elapsed =
      std::chrono::duration_cast<std::chrono::milliseconds>(now - fps_window_begin).count();
    if (fps_elapsed >= 1000) {
      publish_fps = fps_window_frames * 1000.0 / static_cast<double>(fps_elapsed);
      // tools::logger()->info("AutoAim FPS: {:.1f}", publish_fps);
      fps_window_begin = now;
      fps_window_frames = 0;
    }

    std::optional<cv::Point> aim_point_px = std::nullopt;
    std::optional<double> final_x = std::nullopt;
    if (aimer.debug_aim_point.valid) {
      auto aim_xyz = aimer.debug_aim_point.xyza.head(3);
      final_x = aim_xyz.x();
      std::vector<cv::Point3f> world_points = {cv::Point3f(
        static_cast<float>(aim_xyz.x()), static_cast<float>(aim_xyz.y()),
        static_cast<float>(aim_xyz.z()))};
      auto projected = solver.world2pixel(world_points);
      if (!projected.empty()) {
        aim_point_px =
          cv::Point(static_cast<int>(projected.front().x), static_cast<int>(projected.front().y));
      }
    }

    cv::Mat autoaim_img = img.clone();
    draw_autoaim_overlay(
      autoaim_img, armors, command, tracker.state(), search_color, self_color, publish_fps,
      aim_point_px, final_x);
    ros2.publish_autoaim_image(autoaim_img);

    /// ROS2通信
    Eigen::Vector4d target_info = decider.get_target_info(armors, targets);

    ros2.publish(target_info);
  }
  return 0;
}