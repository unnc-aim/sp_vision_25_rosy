#ifndef TOOLS__PROFILE_LOG_HPP
#define TOOLS__PROFILE_LOG_HPP

#include <chrono>
#include <cstddef>
#include <fstream>
#include <functional>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace tools
{
struct ProfileLogOutputConfig
{
  bool stdout_enabled = false;
  bool file_enabled = true;
  bool ros2_topic_enabled = false;
};

class ProfileLog
{
public:
  ProfileLog(
    const std::string & name = "profile", std::size_t flush_every = 200, bool enabled = true);

  ProfileLog(
    const std::string & name, std::size_t flush_every, bool enabled,
    const ProfileLogOutputConfig & output_config,
    std::function<void(const std::string &)> ros2_topic_publisher = nullptr);

  ~ProfileLog();

  void next_frame();
  void record(const std::string & module, double cost_ms);

  bool enabled() const;

private:
  bool enabled_;
  ProfileLogOutputConfig output_config_;
  std::size_t frame_id_;
  std::size_t flush_every_;
  std::size_t pending_lines_;
  std::string file_path_;
  std::ofstream writer_;
  std::mutex mtx_;
  std::function<void(const std::string &)> ros2_topic_publisher_;
  bool frame_started_;
  std::vector<std::pair<std::string, double>> current_frame_records_;
  std::vector<std::string> pending_frame_lines_;

  void flush_if_needed();
  void flush_pending(bool force);
  void finalize_current_frame();
};

class ProfileScope
{
public:
  ProfileScope(ProfileLog & profile_log, const std::string & module);
  ~ProfileScope();

private:
  ProfileLog & profile_log_;
  std::string module_;
  std::chrono::steady_clock::time_point begin_;
};

}  // namespace tools

#endif  // TOOLS__PROFILE_LOG_HPP
