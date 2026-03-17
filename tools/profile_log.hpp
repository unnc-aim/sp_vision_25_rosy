#ifndef TOOLS__PROFILE_LOG_HPP
#define TOOLS__PROFILE_LOG_HPP

#include <chrono>
#include <cstddef>
#include <fstream>
#include <mutex>
#include <string>

namespace tools
{
class ProfileLog
{
public:
  ProfileLog(
    const std::string & name = "profile", std::size_t flush_every = 200, bool enabled = true);
  ~ProfileLog();

  void next_frame();
  void record(const std::string & module, double cost_ms);

  bool enabled() const;

private:
  bool enabled_;
  std::size_t frame_id_;
  std::size_t flush_every_;
  std::size_t pending_lines_;
  std::ofstream writer_;
  std::mutex mtx_;

  void flush_if_needed();
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
