#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <chrono>
#include <cmath>
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
#include "tools/profile_log.hpp"
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
  const std::string & self_color, double fps, double font_scale,
  const std::optional<cv::Point> & aim_point_px, const std::optional<double> & final_x)
{
  const double safe_font_scale = std::max(0.1, font_scale);

  double max_confidence = -1.0;
  for (const auto & armor : armors) {
    max_confidence = std::max(max_confidence, armor.confidence);
  }

  const auto confidence_text =
    max_confidence >= 0.0 ? fmt::format("confidence={:.0f}%", max_confidence * 100.0)
                          : std::string("confidence=n/a");

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

    auto confidence_text = fmt::format("conf={:.0f}%", armor.confidence * 100.0);
    auto confidence_pos = cv::Point(armor.box.x, std::max(20, armor.box.y - 30));
    auto confidence_size = cv::getTextSize(
      confidence_text, cv::FONT_HERSHEY_SIMPLEX, 0.6 * safe_font_scale, 2, nullptr);
    cv::Rect confidence_box(
      confidence_pos.x, confidence_pos.y - confidence_size.height - 6,
      confidence_size.width + 10, confidence_size.height + 10);
    cv::rectangle(image, confidence_box, cv::Scalar(0, 255, 255), cv::FILLED);
    tools::draw_text(
      image, confidence_text, {confidence_pos.x + 5, confidence_pos.y}, {0, 0, 0},
      0.6 * safe_font_scale, 2);

    auto label = fmt::format(
      "{}:{} {:.2f}", auto_aim::COLORS[armor.color], auto_aim::ARMOR_NAMES[armor.name],
      armor.confidence);
    cv::Point text_pos{armor.box.x, std::max(20, armor.box.y - 8)};
    tools::draw_text(image, label, text_pos, {0, 255, 0}, 0.6 * safe_font_scale, 2);
  }

  auto color_status = fmt::format("self_color={} search_color={}", self_color, search_color);
  tools::draw_text(image, color_status, {10, static_cast<int>(std::lround(15.0 * safe_font_scale))},
    {255, 255, 0}, 0.75 * safe_font_scale, 2);

  auto status = fmt::format(
    "state={} ctrl={} shoot={} yaw={:.3f} pitch={:.3f}", tracker_state, command.control,
    command.shoot, command.yaw, command.pitch);
  tools::draw_text(image, status,
    {10, static_cast<int>(std::lround(32.0 * safe_font_scale))}, {0, 255, 255},
    0.65 * safe_font_scale, 2);

  auto fps_text = fmt::format("fps={:.1f}", fps);
  tools::draw_text(image, fps_text, {10, static_cast<int>(std::lround(48.0 * safe_font_scale))},
    {255, 255, 255}, 0.65 * safe_font_scale, 2);
  tools::draw_text(image, confidence_text,
    {10, static_cast<int>(std::lround(65.0 * safe_font_scale))}, {0, 255, 255},
    0.7 * safe_font_scale, 2);

  // if (final_x.has_value()) {
  //   auto final_x_text = fmt::format("final_x={:.3f}", final_x.value());
  //   tools::draw_text(image, final_x_text, {10, 120}, {255, 255, 255}, 0.7, 2);
  // }

  if (aim_point_px.has_value()) {
    auto p = aim_point_px.value();
    cv::drawMarker(image, p, cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 20, 2);
    tools::draw_text(image, "aim_point", {p.x + 8, p.y - 8}, {0, 0, 255}, 0.6 * safe_font_scale, 2);
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

  bool profile_log_enabled = true;
  if (yaml["profile_log_enabled"]) {
    profile_log_enabled = yaml["profile_log_enabled"].as<bool>();
  }

  bool enable_ros2_image_publish = true;
  if (yaml["enable_ros2_image_publish"]) {
    enable_ros2_image_publish = yaml["enable_ros2_image_publish"].as<bool>();
  }

  double overlay_font_scale = 2.0;
  if (yaml["overlay_font_scale"]) {
    overlay_font_scale = yaml["overlay_font_scale"].as<double>();
  }
  overlay_font_scale = std::max(0.1, overlay_font_scale);

  std::size_t profile_log_flush_every = 200;
  if (yaml["profile_log_flush_every"]) {
    profile_log_flush_every = yaml["profile_log_flush_every"].as<std::size_t>();
  }

  bool profile_log_stdout_enabled = false;
  if (yaml["profile_log_stdout_enabled"]) {
    profile_log_stdout_enabled = yaml["profile_log_stdout_enabled"].as<bool>();
  }

  bool profile_log_file_enabled = true;
  if (yaml["profile_log_file_enabled"]) {
    profile_log_file_enabled = yaml["profile_log_file_enabled"].as<bool>();
  }

  bool profile_log_ros2_topic_enabled = false;
  if (yaml["profile_log_ros2_topic_enabled"]) {
    profile_log_ros2_topic_enabled = yaml["profile_log_ros2_topic_enabled"].as<bool>();
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

  tools::ProfileLogOutputConfig profile_log_output_config;
  profile_log_output_config.stdout_enabled = profile_log_stdout_enabled;
  profile_log_output_config.file_enabled = profile_log_file_enabled;
  profile_log_output_config.ros2_topic_enabled = profile_log_ros2_topic_enabled;

  tools::ProfileLog profile_log(
    "sentry_profile", profile_log_flush_every, profile_log_enabled, profile_log_output_config,
    [&ros2](const std::string & line) { ros2.publish_profile_log(line); });

  omniperception::Decider decider(config_path);

  // Pitch 命令保护参数：抑制目标角突跳导致的异常状态。
  bool pitch_guard_enabled = true;
  if (yaml["pitch_guard_enabled"]) {
    pitch_guard_enabled = yaml["pitch_guard_enabled"].as<bool>();
  }
  double pitch_guard_max_error_deg = 8.0;
  if (yaml["pitch_guard_max_error_deg"]) {
    pitch_guard_max_error_deg = yaml["pitch_guard_max_error_deg"].as<double>();
  }
  double pitch_cmd_max_step_deg = 1.5;
  if (yaml["pitch_cmd_max_step_deg"]) {
    pitch_cmd_max_step_deg = yaml["pitch_cmd_max_step_deg"].as<double>();
  }
  double pitch_cmd_min_deg = -25.0;
  if (yaml["pitch_cmd_min_deg"]) {
    pitch_cmd_min_deg = yaml["pitch_cmd_min_deg"].as<double>();
  }
  double pitch_cmd_max_deg = 40.0;
  if (yaml["pitch_cmd_max_deg"]) {
    pitch_cmd_max_deg = yaml["pitch_cmd_max_deg"].as<double>();
  }

  const double pitch_guard_max_error_rad = pitch_guard_max_error_deg / 57.3;
  const double pitch_cmd_max_step_rad = std::max(0.0, pitch_cmd_max_step_deg / 57.3);
  const double pitch_cmd_min_rad = pitch_cmd_min_deg / 57.3;
  const double pitch_cmd_max_rad = pitch_cmd_max_deg / 57.3;
  bool last_pitch_cmd_valid = false;
  double last_pitch_cmd = 0.0;
  int pitch_guard_log_counter = 0;

  cv::Mat img;

  std::chrono::steady_clock::time_point timestamp;
  auto fps_window_begin = std::chrono::steady_clock::now();
  int fps_window_frames = 0;
  double publish_fps = 0.0;
  io::Command last_command;
  int autoaim_log_counter = 0;  // <<<< AUTOAIM DEBUG LOG COUNTER >>>>

  while (!exiter.exit()) {
    profile_log.next_frame();
    tools::ProfileScope loop_scope(profile_log, "loop.total");

    {
      tools::ProfileScope scope(profile_log, "camera.read");
      camera.read(img, timestamp);
    }

    if (img.empty()) {
      cv::Mat placeholder(720, 1280, CV_8UC3, cv::Scalar(0, 0, 0));
      tools::draw_text(placeholder, "SP Vision: no camera frame", {30, 60}, {0, 0, 255}, 3.0, 2);
      if (enable_ros2_image_publish) {
        {
          tools::ProfileScope scope(profile_log, "ros2.publish_raw_image");
          ros2.publish_raw_image(placeholder);
        }
        {
          tools::ProfileScope scope(profile_log, "ros2.publish_autoaim_image");
          ros2.publish_autoaim_image(placeholder);
        }
      }
      continue;
    }

    if (enable_ros2_image_publish) {
      tools::ProfileScope scope(profile_log, "ros2.publish_raw_image");
      ros2.publish_raw_image(img);
    }

    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    if (cboard) {
      tools::ProfileScope scope(profile_log, "cboard.imu_at");
      q = cboard->imu_at(timestamp - 1ms);
    } else {
      tools::ProfileScope scope(profile_log, "ros2.subscribe_imu");
      q = ros2.subscribe_imu();
    }
    // recorder.record(img, q, timestamp);

    // >>>>>>>>>>>> AUTOAIM DEBUG: IMU quaternion <<<<<<<<<<<<
    static int imu_log_counter = 0;
    if (++imu_log_counter >= 200) {  // ~1Hz at ~200fps
      imu_log_counter = 0;
      auto q_norm = q.norm();
      tools::logger()->info(
        "[AUTOAIM-IMU] q=({:.4f},{:.4f},{:.4f},{:.4f}) norm={:.4f}", q.w(), q.x(), q.y(), q.z(),
        q_norm);
    }
    // <<<<<<<<<<<<< AUTOAIM DEBUG END >>>>>>>>>>>>>>>>>>>>>>>>

    /// 自瞄核心逻辑
    {
      tools::ProfileScope scope(profile_log, "solver.set_R_gimbal2world");
      solver.set_R_gimbal2world(q);
    }

    Eigen::Vector3d gimbal_pos;
    {
      tools::ProfileScope scope(profile_log, "tools.eulers");
      gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
    }

    std::list<auto_aim::Armor> armors;
    {
      tools::ProfileScope scope(profile_log, "detector.yolo.detect");
      armors = yolo.detect(img);
    }
    {
      tools::ProfileScope scope(profile_log, "ros2.subscribe_self_color");
      decider.set_self_color(ros2.subscribe_self_color());
    }

    {
      tools::ProfileScope scope(profile_log, "ros2.subscribe_enemy_status");
      decider.get_invincible_armor(ros2.subscribe_enemy_status());
    }

    {
      tools::ProfileScope scope(profile_log, "decider.armor_filter");
      decider.armor_filter(armors);
    }
    // decider.get_auto_aim_target(armors, ros2.subscribe_autoaim_target());
    {
      tools::ProfileScope scope(profile_log, "decider.set_priority");
      decider.set_priority(armors);
    }

    std::list<auto_aim::Target> targets;
    {
      tools::ProfileScope scope(profile_log, "tracker.track");
      targets = tracker.track(armors, timestamp);
    }

    io::Command command{false, false, 0, 0};

    /// 全向感知逻辑
    if (tracker.state() == "lost") {
      if (use_usb_cameras && usbcam1 && usbcam2) {
        tools::ProfileScope scope(profile_log, "decider.decide.multi_camera");
        command = decider.decide(
          yolo, gimbal_pos, *usbcam1, *usbcam2, back_camera ? *back_camera : camera,
          use_back_camera);
      } else {
        tools::ProfileScope scope(profile_log, "decider.decide.single_camera");
        command =
          decider.decide(yolo, gimbal_pos, back_camera ? *back_camera : camera, use_back_camera);
      }
    } else {
      tools::ProfileScope scope(profile_log, "aimer.aim");
      command = aimer.aim(
        targets, timestamp, cboard ? cboard->bullet_speed : 0.0,
        cboard ? cboard->shoot_mode : io::ShootMode::left_shoot);
    }

    // Pitch 命令保护：先做角差限幅，再做单帧限速，最后做绝对俯仰限位。
    if (pitch_guard_enabled && command.control) {
      double pitch_err = command.pitch - gimbal_pos[1];
      if (std::fabs(pitch_err) > pitch_guard_max_error_rad) {
        command.pitch =
          gimbal_pos[1] + (pitch_err > 0 ? pitch_guard_max_error_rad : -pitch_guard_max_error_rad);
        pitch_guard_log_counter++;
        if (pitch_guard_log_counter >= 20) {
          pitch_guard_log_counter = 0;
          tools::logger()->warn(
            "[PITCH-GUARD] large error clipped: err={:.2f}deg limit={:.2f}deg tracker={}",
            pitch_err * 57.3, pitch_guard_max_error_deg, tracker.state());
        }
      }

      if (!last_pitch_cmd_valid) {
        last_pitch_cmd = command.pitch;
        last_pitch_cmd_valid = true;
      } else {
        double step = command.pitch - last_pitch_cmd;
        if (std::fabs(step) > pitch_cmd_max_step_rad) {
          command.pitch =
            last_pitch_cmd + (step > 0 ? pitch_cmd_max_step_rad : -pitch_cmd_max_step_rad);
        }
      }

      command.pitch = tools::limit_min_max(command.pitch, pitch_cmd_min_rad, pitch_cmd_max_rad);
      last_pitch_cmd = command.pitch;
    }

    /// 发射逻辑
    {
      tools::ProfileScope scope(profile_log, "shooter.shoot");
      command.shoot = shooter.shoot(command, aimer, targets, gimbal_pos);
    }

    // >>>>>>>>>>>> AUTOAIM DEBUG: command output <<<<<<<<<<<<
    if (command.control) {
      autoaim_log_counter++;
      if (autoaim_log_counter >= 20) {  // ~10Hz at ~200fps
        autoaim_log_counter = 0;
        auto aim_xyz = aimer.debug_aim_point.xyza.head(3);
        tools::logger()->info(
          "[AUTOAIM-CMD] yaw={:.4f} pitch={:.4f} shoot={} | "
          "aim_xyz=({:.3f},{:.3f},{:.3f}) | tracker={} | armors={}",
          command.yaw, command.pitch, command.shoot, aim_xyz.x(), aim_xyz.y(), aim_xyz.z(),
          tracker.state(), armors.size());
      }
    } else {
      autoaim_log_counter = 0;
    }
    // <<<<<<<<<<<<< AUTOAIM DEBUG END >>>>>>>>>>>>>>>>>>>>>>>>

    if (cboard) {
      tools::ProfileScope scope(profile_log, "cboard.send");
      cboard->send(command);
    }
    {
      tools::ProfileScope scope(profile_log, "ros2.publish_autoaim_command");
      ros2.publish_autoaim_command(command);
    }

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

    cv::Mat autoaim_img;
    {
      tools::ProfileScope scope(profile_log, "image.clone");
      autoaim_img = img.clone();
    }
    {
      tools::ProfileScope scope(profile_log, "draw_autoaim_overlay");
      draw_autoaim_overlay(
        autoaim_img, armors, command, tracker.state(), search_color, self_color, publish_fps,
        overlay_font_scale, aim_point_px, final_x);
    }
    if (enable_ros2_image_publish) {
      tools::ProfileScope scope(profile_log, "ros2.publish_autoaim_image");
      ros2.publish_autoaim_image(autoaim_img);
    }

    /// ROS2通信
    Eigen::Vector4d target_info;
    {
      tools::ProfileScope scope(profile_log, "decider.get_target_info");
      target_info = decider.get_target_info(armors, targets);
    }

    {
      tools::ProfileScope scope(profile_log, "ros2.publish.target_info");
      ros2.publish(target_info);
    }
  }
  return 0;
}