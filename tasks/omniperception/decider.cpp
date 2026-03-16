#include "decider.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <opencv2/opencv.hpp>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace omniperception
{
Decider::Decider(const std::string & config_path)
: aim_ally_(false),
  aim_enemy_(true),
  aim_red_(false),
  aim_blue_(false),
  detector_(config_path),
  count_(0)
{
  auto yaml = YAML::LoadFile(config_path);
  img_width_ = yaml["image_width"].as<double>();
  img_height_ = yaml["image_height"].as<double>();
  fov_h_ = yaml["fov_h"].as<double>();
  fov_v_ = yaml["fov_v"].as<double>();
  new_fov_h_ = yaml["new_fov_h"].as<double>();
  new_fov_v_ = yaml["new_fov_v"].as<double>();

  // 四个开关是“或”关系，只要命中任一条件就允许该颜色。
  if (yaml["auto_aim_ally"]) aim_ally_ = yaml["auto_aim_ally"].as<bool>();
  if (yaml["auto_aim_enemy"]) aim_enemy_ = yaml["auto_aim_enemy"].as<bool>();
  if (yaml["auto_aim_red"]) aim_red_ = yaml["auto_aim_red"].as<bool>();
  if (yaml["auto_aim_blue"]) aim_blue_ = yaml["auto_aim_blue"].as<bool>();

  mode_ = yaml["mode"].as<double>();
}

void Decider::set_self_color(const std::string & self_color)
{
  if (self_color == "red") {
    self_color_ = auto_aim::Color::red;
  } else if (self_color == "blue") {
    self_color_ = auto_aim::Color::blue;
  } else {
    self_color_.reset();
  }
}

void Decider::compute_allowed_colors(bool & allow_red, bool & allow_blue) const
{
  allow_red = false;
  allow_blue = false;

  if (aim_red_) allow_red = true;
  if (aim_blue_) allow_blue = true;

  if (self_color_.has_value()) {
    if (aim_ally_) {
      if (self_color_.value() == auto_aim::Color::red) {
        allow_red = true;
      } else if (self_color_.value() == auto_aim::Color::blue) {
        allow_blue = true;
      }
    }
    if (aim_enemy_) {
      if (self_color_.value() == auto_aim::Color::red) {
        allow_blue = true;
      } else if (self_color_.value() == auto_aim::Color::blue) {
        allow_red = true;
      }
    }
  }
}

std::string Decider::current_search_color_text() const
{
  bool allow_red = false;
  bool allow_blue = false;
  compute_allowed_colors(allow_red, allow_blue);
  if (allow_red && allow_blue) return "red|blue";
  if (allow_red) return "red";
  if (allow_blue) return "blue";
  return "none";
}

std::string Decider::current_self_color_text() const
{
  if (!self_color_.has_value()) return "unknown";
  return self_color_.value() == auto_aim::Color::red ? "red" : "blue";
}

bool Decider::is_color_allowed(auto_aim::Color armor_color) const
{
  bool allow_red = false;
  bool allow_blue = false;
  compute_allowed_colors(allow_red, allow_blue);

  if (armor_color == auto_aim::Color::red) return allow_red;
  if (armor_color == auto_aim::Color::blue) return allow_blue;
  return false;
}

io::Command Decider::decide(
  auto_aim::YOLO & yolo, const Eigen::Vector3d & gimbal_pos, io::USBCamera & usbcam1,
  io::USBCamera & usbcam2, io::Camera & back_camera, bool use_back_camera)
{
  Eigen::Vector2d delta_angle;
  io::USBCamera * cams[] = {&usbcam1, &usbcam2};

  cv::Mat usb_img;
  std::chrono::steady_clock::time_point timestamp;
  if (count_ < 0 || count_ > 2) {
    throw std::runtime_error("count_ out of valid range [0,2]");
  }
  if (count_ == 2) {
    back_camera.read(usb_img, timestamp);
  } else {
    cams[count_]->read(usb_img, timestamp);
  }
  auto armors = yolo.detect(usb_img);
  auto empty = armor_filter(armors);

  if (!empty) {
    std::string camera_name;
    if (count_ == 2) {
      camera_name = use_back_camera ? "back" : "front";
      delta_angle = this->delta_angle(armors, camera_name);
    } else {
      camera_name = cams[count_]->device_name;
      delta_angle = this->delta_angle(armors, camera_name);
    }

    tools::logger()->debug(
      "[{} camera] delta yaw:{:.2f},target pitch:{:.2f},armor number:{},armor name:{}", camera_name,
      delta_angle[0], delta_angle[1], armors.size(), auto_aim::ARMOR_NAMES[armors.front().name]);

    count_ = (count_ + 1) % 3;

    return io::Command{
      true, false, tools::limit_rad(gimbal_pos[0] + delta_angle[0] / 57.3),
      tools::limit_rad(delta_angle[1] / 57.3)};
  }

  count_ = (count_ + 1) % 3;
  // 如果没有找到目标，返回默认命令
  return io::Command{false, false, 0, 0};
}

io::Command Decider::decide(
  auto_aim::Detector & detector, const Eigen::Vector3d & gimbal_pos, io::USBCamera & usbcam1,
  io::USBCamera & usbcam2, io::Camera & back_camera, bool use_back_camera)
{
  Eigen::Vector2d delta_angle;
  io::USBCamera * cams[] = {&usbcam1, &usbcam2};

  cv::Mat usb_img;
  std::chrono::steady_clock::time_point timestamp;
  if (count_ < 0 || count_ > 2) {
    throw std::runtime_error("count_ out of valid range [0,2]");
  }
  if (count_ == 2) {
    back_camera.read(usb_img, timestamp);
  } else {
    cams[count_]->read(usb_img, timestamp);
  }
  auto armors = detector.detect(usb_img);
  auto empty = armor_filter(armors);

  if (!empty) {
    std::string camera_name;
    if (count_ == 2) {
      camera_name = use_back_camera ? "back" : "front";
      delta_angle = this->delta_angle(armors, camera_name);
    } else {
      camera_name = cams[count_]->device_name;
      delta_angle = this->delta_angle(armors, camera_name);
    }

    tools::logger()->debug(
      "[{} camera] delta yaw:{:.2f},target pitch:{:.2f},armor number:{},armor name:{}", camera_name,
      delta_angle[0], delta_angle[1], armors.size(), auto_aim::ARMOR_NAMES[armors.front().name]);

    count_ = (count_ + 1) % 3;

    return io::Command{
      true, false, tools::limit_rad(gimbal_pos[0] + delta_angle[0] / 57.3),
      tools::limit_rad(delta_angle[1] / 57.3)};
  }

  count_ = (count_ + 1) % 3;
  return io::Command{false, false, 0, 0};
}

io::Command Decider::decide(
  auto_aim::YOLO & yolo, const Eigen::Vector3d & gimbal_pos, io::Camera & back_cammera,
  bool use_back_camera)
{
  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  back_cammera.read(img, timestamp);
  auto armors = yolo.detect(img);
  auto empty = armor_filter(armors);

  if (!empty) {
    auto camera_name = use_back_camera ? std::string("back") : std::string("front");
    auto delta_angle = this->delta_angle(armors, camera_name);
    tools::logger()->debug(
      "[{} camera] delta yaw:{:.2f},target pitch:{:.2f},armor number:{},armor name:{}", camera_name,
      delta_angle[0], delta_angle[1], armors.size(), auto_aim::ARMOR_NAMES[armors.front().name]);

    return io::Command{
      true, false, tools::limit_rad(gimbal_pos[0] + delta_angle[0] / 57.3),
      tools::limit_rad(delta_angle[1] / 57.3)};
  }

  return io::Command{false, false, 0, 0};
}

io::Command Decider::decide(
  auto_aim::Detector & detector, const Eigen::Vector3d & gimbal_pos, io::Camera & back_cammera,
  bool use_back_camera)
{
  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  back_cammera.read(img, timestamp);
  auto armors = detector.detect(img);
  auto empty = armor_filter(armors);

  if (!empty) {
    auto camera_name = use_back_camera ? std::string("back") : std::string("front");
    auto delta_angle = this->delta_angle(armors, camera_name);
    tools::logger()->debug(
      "[{} camera] delta yaw:{:.2f},target pitch:{:.2f},armor number:{},armor name:{}", camera_name,
      delta_angle[0], delta_angle[1], armors.size(), auto_aim::ARMOR_NAMES[armors.front().name]);

    return io::Command{
      true, false, tools::limit_rad(gimbal_pos[0] + delta_angle[0] / 57.3),
      tools::limit_rad(delta_angle[1] / 57.3)};
  }

  return io::Command{false, false, 0, 0};
}

io::Command Decider::decide(const std::vector<DetectionResult> & detection_queue)
{
  if (detection_queue.empty()) {
    return io::Command{false, false, 0, 0};
  }

  DetectionResult dr = detection_queue.front();
  if (dr.armors.empty()) return io::Command{false, false, 0, 0};
  tools::logger()->info(
    "omniperceptron find {},delta yaw is {:.4f}", auto_aim::ARMOR_NAMES[dr.armors.front().name],
    dr.delta_yaw * 57.3);

  return io::Command{true, false, dr.delta_yaw, dr.delta_pitch};
};

Eigen::Vector2d Decider::delta_angle(
  const std::list<auto_aim::Armor> & armors, const std::string & camera)
{
  Eigen::Vector2d delta_angle;
  if (camera == "left") {
    delta_angle[0] = 62 + (new_fov_h_ / 2) - armors.front().center_norm.x * new_fov_h_;
    delta_angle[1] = armors.front().center_norm.y * new_fov_v_ - new_fov_v_ / 2;
    return delta_angle;
  }

  else if (camera == "right") {
    delta_angle[0] = -62 + (new_fov_h_ / 2) - armors.front().center_norm.x * new_fov_h_;
    delta_angle[1] = armors.front().center_norm.y * new_fov_v_ - new_fov_v_ / 2;
    return delta_angle;
  }

  else if (camera == "front") {
    delta_angle[0] = (fov_h_ / 2) - armors.front().center_norm.x * fov_h_;
    delta_angle[1] = armors.front().center_norm.y * fov_v_ - fov_v_ / 2;
    return delta_angle;
  }

  else {
    delta_angle[0] = 170 + (54.2 / 2) - armors.front().center_norm.x * 54.2;
    delta_angle[1] = armors.front().center_norm.y * 44.5 - 44.5 / 2;
    return delta_angle;
  }
}

bool Decider::armor_filter(std::list<auto_aim::Armor> & armors)
{
  if (armors.empty()) return true;
  // 根据四个bool配置（或关系）过滤装甲板颜色。
  armors.remove_if([&](const auto_aim::Armor & a) { return !is_color_allowed(a.color); });

  // 25赛季没有5号装甲板
  armors.remove_if([&](const auto_aim::Armor & a) { return a.name == auto_aim::ArmorName::five; });
  // 不打工程
  // armors.remove_if([&](const auto_aim::Armor & a) { return a.name == auto_aim::ArmorName::two; });
  // 不打前哨站
  armors.remove_if(
    [&](const auto_aim::Armor & a) { return a.name == auto_aim::ArmorName::outpost; });

  // 过滤掉刚复活无敌的装甲板
  armors.remove_if([&](const auto_aim::Armor & a) {
    return std::find(invincible_armor_.begin(), invincible_armor_.end(), a.name) !=
           invincible_armor_.end();
  });

  return armors.empty();
}

void Decider::set_priority(std::list<auto_aim::Armor> & armors)
{
  if (armors.empty()) return;

  const PriorityMap & priority_map = (mode_ == MODE_ONE) ? mode1 : mode2;

  if (!armors.empty()) {
    for (auto & armor : armors) {
      armor.priority = priority_map.at(armor.name);
    }
  }
}

void Decider::sort(std::vector<DetectionResult> & detection_queue)
{
  if (detection_queue.empty()) return;

  // 对每个 DetectionResult 调用 armor_filter 和 set_priority
  for (auto & dr : detection_queue) {
    armor_filter(dr.armors);
    set_priority(dr.armors);

    // 对每个 DetectionResult 中的 armors 进行排序
    dr.armors.sort(
      [](const auto_aim::Armor & a, const auto_aim::Armor & b) { return a.priority < b.priority; });
  }

  // 根据优先级对 DetectionResult 进行排序
  std::sort(
    detection_queue.begin(), detection_queue.end(),
    [](const DetectionResult & a, const DetectionResult & b) {
      return a.armors.front().priority < b.armors.front().priority;
    });
}

Eigen::Vector4d Decider::get_target_info(
  const std::list<auto_aim::Armor> & armors, const std::list<auto_aim::Target> & targets)
{
  if (armors.empty() || targets.empty()) return Eigen::Vector4d::Zero();

  auto target = targets.front();

  for (const auto & armor : armors) {
    if (armor.name == target.name) {
      return Eigen::Vector4d{
        armor.xyz_in_gimbal[0], armor.xyz_in_gimbal[1], 1,
        static_cast<double>(armor.name) + 1};  //避免歧义+1(详见通信协议)
    }
  }

  return Eigen::Vector4d::Zero();
}

void Decider::get_invincible_armor(const std::vector<int8_t> & invincible_enemy_ids)
{
  invincible_armor_.clear();

  if (invincible_enemy_ids.empty()) return;

  for (const auto & id : invincible_enemy_ids) {
    tools::logger()->info("invincible armor id: {}", id);
    invincible_armor_.push_back(auto_aim::ArmorName(id - 1));
  }
}

void Decider::get_auto_aim_target(
  std::list<auto_aim::Armor> & armors, const std::vector<int8_t> & auto_aim_target)
{
  if (auto_aim_target.empty()) return;

  std::vector<auto_aim::ArmorName> auto_aim_targets;

  for (const auto & target : auto_aim_target) {
    if (target <= 0 || static_cast<size_t>(target) > auto_aim::ARMOR_NAMES.size()) {
      tools::logger()->warn("Received invalid auto_aim target value: {}", int(target));
      continue;
    }
    auto_aim_targets.push_back(static_cast<auto_aim::ArmorName>(target - 1));
    tools::logger()->info("nav send auto_aim target is {}", auto_aim::ARMOR_NAMES[target - 1]);
  }

  if (auto_aim_targets.empty()) return;

  armors.remove_if([&](const auto_aim::Armor & a) {
    return std::find(auto_aim_targets.begin(), auto_aim_targets.end(), a.name) ==
           auto_aim_targets.end();
  });
}

}  // namespace omniperception