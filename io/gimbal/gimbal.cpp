#include "gimbal.hpp"

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{
Gimbal::Gimbal(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");
  unsigned long baudrate = 115200;
  if (yaml["gimbal_baud"]) baudrate = yaml["gimbal_baud"].as<unsigned long>();

  uint8_t header = 0x5A;
  if (yaml["gimbal_header"]) header = yaml["gimbal_header"].as<int>();
  rx_data_.header = header;
  tx_data_.header = header;

  try {
    serial_.setPort(com_port);
    serial_.setBaudrate(baudrate);
    serial_.setFlowcontrol(serial::flowcontrol_none);
    serial_.setParity(serial::parity_none);
    serial_.setStopbits(serial::stopbits_one);
    serial_.setBytesize(serial::eightbits);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(50);
    serial_.setTimeout(timeout);
    serial_.open();
  } catch (const std::exception & e) {
    tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
    exit(1);
  }

  thread_ = std::thread(&Gimbal::read_thread, this);

  queue_.pop();
  tools::logger()->info("[Gimbal] First q received.");
}

Gimbal::~Gimbal()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  serial_.close();
}

GimbalMode Gimbal::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

GimbalState Gimbal::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

std::string Gimbal::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::IDLE:
      return "IDLE";
    case GimbalMode::AUTO_AIM:
      return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF:
      return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF:
      return "BIG_BUFF";
    default:
      return "INVALID";
  }
}

Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t)
{
  while (true) {
    auto [q_a, t_a] = queue_.pop();
    auto [q_b, t_b] = queue_.front();
    auto t_ab = tools::delta_time(t_a, t_b);
    auto t_ac = tools::delta_time(t_a, t);
    auto k = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();
    if (t < t_a) return q_c;
    if (!(t_a < t && t <= t_b)) continue;

    return q_c;
  }
}

void Gimbal::send(io::VisionToGimbal VisionToGimbal)
{
  // tx_data_.mode = VisionToGimbal.mode;
  // tx_data_.yaw = VisionToGimbal.yaw;
  // tx_data_.yaw_vel = VisionToGimbal.yaw_vel;
  // tx_data_.yaw_acc = VisionToGimbal.yaw_acc;
  // tx_data_.pitch = VisionToGimbal.pitch;
  // tx_data_.pitch_vel = VisionToGimbal.pitch_vel;
  // tx_data_.pitch_acc = VisionToGimbal.pitch_acc;
  // tx_data_.crc16 = tools::get_crc16(
  //   reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));
  VisionToGimbal.header = 0xA5;
  VisionToGimbal.checksum = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&VisionToGimbal), sizeof(VisionToGimbal) - sizeof(VisionToGimbal.checksum));

  try {
    //serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
    serial_.write(reinterpret_cast<uint8_t *>(&VisionToGimbal), sizeof(VisionToGimbal));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

void Gimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc)
{

  io::VisionToGimbal packet{};
  packet.header = 0xA5;
  packet.tracking = 1;
  packet.id = 0;
  packet.armors_num = 0;
  packet.x = 0.0f;
  packet.y = 0.0f;
  packet.z = 0.0f;
  packet.vx = 0.0f;
  packet.vy = 0.0f;
  packet.vz = 0.0f;
  packet.r1 = 0.0f;
  packet.r2 = 0.0f;
  packet.checksum = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&packet), sizeof(packet) - sizeof(packet.checksum));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

bool Gimbal::read(uint8_t * buffer, size_t size)
{
  try {
    return serial_.read(buffer, size) == size;
  } catch (const std::exception & e) {
    // tools::logger()->warn("[Gimbal] Failed to read serial: {}", e.what());
    return false;
  }
}

void Gimbal::read_thread()
{
  tools::logger()->info("[Gimbal] read_thread started.");
  int error_count = 0;

  while (!quit_) {
    if (error_count > 5000) {
      error_count = 0;
      tools::logger()->warn("[Gimbal] Too many errors, attempting to reconnect...");
      reconnect();
      continue;
    }

    // 读取完整数据帧
    if (!read(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(GimbalToVision))) {
      error_count++;
      continue;
    }

    auto t = std::chrono::steady_clock::now();

    // if (!read(
    //       reinterpret_cast<uint8_t *>(&rx_data_) + sizeof(rx_data_.header),
    //       sizeof(rx_data_) - sizeof(rx_data_.header))) {
    //   error_count++;
    //   continue;
    // }

    if (!tools::check_crc16(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(GimbalToVision))) {
      tools::logger()->debug("[Gimbal] CRC16 check failed.");
      error_count++;
      continue;
    }

    error_count = 0;

    // roll/pitch/yaw -> 四元数
    Eigen::Quaterniond q =
      Eigen::AngleAxisd(rx_data_.yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(rx_data_.pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(rx_data_.roll, Eigen::Vector3d::UnitX());
    q.normalize();
    queue_.push({q, t});

    std::lock_guard<std::mutex> lock(mutex_);
    state_.yaw = rx_data_.yaw;
    state_.yaw_vel = 0.0f;  
    state_.pitch = rx_data_.pitch;
    state_.pitch_vel = 0.0f; 
    state_.bullet_speed = 0.0f; 
    state_.bullet_count = 0;  

    mode_ = GimbalMode::AUTO_AIM;
  }

  tools::logger()->info("[Gimbal] read_thread stopped.");
}

void Gimbal::reconnect()
{
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[Gimbal] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try {
      serial_.close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (...) {
    }

    try {
      serial_.open();  // 尝试重新打开
      queue_.clear();
      tools::logger()->info("[Gimbal] Reconnected serial successfully.");
      break;
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

}  // namespace io