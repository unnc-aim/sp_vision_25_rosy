#include <fmt/core.h>

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cstdlib>
#include <memory>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{@config-path   | configs/sentry.yaml    | yaml配置文件的路径}"
  "{tradition t    |  false                 | 是否使用传统方法识别}"
  "{display d      |                        | 显示检测图像窗口}";

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);
  auto use_tradition = cli.has("tradition");
  auto display = cli.has("display");
  if (display && std::getenv("DISPLAY") == nullptr) {
    tools::logger()->warn("DISPLAY is not set, fallback to headless mode.");
    display = false;
  }
  if (!use_tradition) {
    auto yaml = YAML::LoadFile(config_path);
    use_tradition = yaml["use_traditional"] ? yaml["use_traditional"].as<bool>() : false;
  }

  tools::Exiter exiter;

  io::Camera camera(config_path);
  std::unique_ptr<auto_aim::Detector> detector;
  std::unique_ptr<auto_aim::YOLO> yolo;
  if (use_tradition)
    detector = std::make_unique<auto_aim::Detector>(config_path, display);
  else
    yolo = std::make_unique<auto_aim::YOLO>(config_path, display);

  std::chrono::steady_clock::time_point timestamp;

  while (!exiter.exit()) {
    cv::Mat img;
    std::list<auto_aim::Armor> armors;

    camera.read(img, timestamp);

    if (img.empty()) break;

    auto last = std::chrono::steady_clock::now();

    if (use_tradition)
      armors = detector->detect(img);
    else
      armors = yolo->detect(img);

    auto now = std::chrono::steady_clock::now();
    auto dt = tools::delta_time(now, last);
    tools::logger()->info("{:.2f} fps", 1 / dt);

    if (display && cv::waitKey(1) == 'q') break;
  }

  return 0;
}