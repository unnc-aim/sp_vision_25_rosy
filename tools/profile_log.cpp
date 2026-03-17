#include "profile_log.hpp"

#include <fmt/chrono.h>

#include <filesystem>

#include "tools/logger.hpp"

namespace tools
{
ProfileLog::ProfileLog(const std::string & name, std::size_t flush_every, bool enabled)
: enabled_(enabled), frame_id_(0), flush_every_(flush_every), pending_lines_(0)
{
  if (!enabled_) return;

  if (flush_every_ == 0) flush_every_ = 1;

  const std::string folder_path = "logs";
  std::filesystem::create_directory(folder_path);

  const auto file_name =
    fmt::format("{}_{:%Y-%m-%d_%H-%M-%S}.csv", name, std::chrono::system_clock::now());
  const auto file_path = fmt::format("{}/{}", folder_path, file_name);

  writer_.open(file_path, std::ios::out);
  if (!writer_.is_open()) {
    tools::logger()->error("[ProfileLog] failed to open profile log file: {}", file_path);
    enabled_ = false;
    return;
  }

  writer_ << "time,frame,module,cost_ms\n";
  writer_.flush();
  tools::logger()->info("[ProfileLog] writing profile log to {}", file_path);
}

ProfileLog::~ProfileLog()
{
  if (!enabled_) return;

  std::lock_guard<std::mutex> lk(mtx_);
  if (writer_.is_open()) {
    writer_.flush();
    writer_.close();
  }
}

void ProfileLog::next_frame()
{
  if (!enabled_) return;
  ++frame_id_;
}

void ProfileLog::record(const std::string & module, double cost_ms)
{
  if (!enabled_) return;

  std::lock_guard<std::mutex> lk(mtx_);
  if (!writer_.is_open()) return;

  const auto now = std::chrono::system_clock::now();
  writer_ << fmt::format("{:%H:%M:%S},{},{},{}\n", now, frame_id_, module, cost_ms);
  ++pending_lines_;
  flush_if_needed();
}

bool ProfileLog::enabled() const { return enabled_; }

void ProfileLog::flush_if_needed()
{
  if (pending_lines_ >= flush_every_) {
    writer_.flush();
    pending_lines_ = 0;
  }
}

ProfileScope::ProfileScope(ProfileLog & profile_log, const std::string & module)
: profile_log_(profile_log), module_(module), begin_(std::chrono::steady_clock::now())
{
}

ProfileScope::~ProfileScope()
{
  if (!profile_log_.enabled()) return;

  const auto end = std::chrono::steady_clock::now();
  const auto us = std::chrono::duration_cast<std::chrono::microseconds>(end - begin_).count();
  const double cost_ms = static_cast<double>(us) / 1000.0;
  profile_log_.record(module_, cost_ms);
}

}  // namespace tools
