#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "io/ros2/ros2.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/multithread/commandgener.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/profile_log.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{@config-path   | configs/standard3.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  auto yaml = YAML::LoadFile(config_path);
  bool profile_log_enabled = true;
  if (yaml["profile_log_enabled"]) {
    profile_log_enabled = yaml["profile_log_enabled"].as<bool>();
  }

  std::size_t profile_log_flush_every = 200;
  if (yaml["profile_log_flush_every"]) {
    profile_log_flush_every = yaml["profile_log_flush_every"].as<std::size_t>();
  }

  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  io::CBoard cboard(config_path);
  io::ROS2 ros2;
  io::Camera camera(config_path);

  auto_aim::YOLO detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  tools::ProfileLog profile_log("standard_profile", profile_log_flush_every, profile_log_enabled);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  auto mode = io::Mode::idle;
  auto last_mode = io::Mode::idle;

  while (!exiter.exit()) {
    profile_log.next_frame();
    tools::ProfileScope loop_scope(profile_log, "loop.total");

    {
      tools::ProfileScope scope(profile_log, "camera.read");
      camera.read(img, t);
    }
    {
      tools::ProfileScope scope(profile_log, "cboard.imu_at");
      q = cboard.imu_at(t - 1ms);
    }
    mode = cboard.mode;

    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", io::MODES[mode]);
      last_mode = mode;
    }

    // recorder.record(img, q, t);

    {
      tools::ProfileScope scope(profile_log, "solver.set_R_gimbal2world");
      solver.set_R_gimbal2world(q);
    }

    Eigen::Vector3d ypr;
    {
      tools::ProfileScope scope(profile_log, "tools.eulers");
      ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
    }

    std::list<auto_aim::Armor> armors;
    {
      tools::ProfileScope scope(profile_log, "detector.yolo.detect");
      armors = detector.detect(img);
    }

    std::list<auto_aim::Target> targets;
    {
      tools::ProfileScope scope(profile_log, "tracker.track");
      targets = tracker.track(armors, t);
    }

    io::Command command;
    {
      tools::ProfileScope scope(profile_log, "aimer.aim");
      command = aimer.aim(targets, t, cboard.bullet_speed);
    }

    {
      tools::ProfileScope scope(profile_log, "cboard.send");
      cboard.send(command);
    }
    {
      tools::ProfileScope scope(profile_log, "ros2.publish_autoaim_command");
      ros2.publish_autoaim_command(command);
    }
  }

  return 0;
}