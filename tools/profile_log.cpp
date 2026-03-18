#include "profile_log.hpp"

#include <fmt/chrono.h>
#include <fmt/format.h>

#include <filesystem>

#include "tools/logger.hpp"

namespace tools
{
ProfileLog::ProfileLog(const std::string & name, std::size_t flush_every, bool enabled)
: ProfileLog(name, flush_every, enabled, ProfileLogOutputConfig{}, nullptr)
{
}

ProfileLog::ProfileLog(
  const std::string & name, std::size_t flush_every, bool enabled,
  const ProfileLogOutputConfig & output_config,
  std::function<void(const std::string &)> ros2_topic_publisher)
: enabled_(enabled),
  output_config_(output_config),
  frame_id_(0),
  flush_every_(flush_every),
  pending_lines_(0),
  ros2_topic_publisher_(std::move(ros2_topic_publisher)),
  frame_started_(false),
  last_frame_time_(std::chrono::steady_clock::now()),
  last_frame_elapsed_ms_(0.0)
{
  if (!enabled_) return;

  if (flush_every_ == 0) flush_every_ = 1;

  if (output_config_.file_enabled) {
    const std::string folder_path = "logs";
    std::filesystem::create_directory(folder_path);

    const auto file_name =
      fmt::format("{}_{:%Y-%m-%d_%H-%M-%S}.csv", name, std::chrono::system_clock::now());
    file_path_ = fmt::format("{}/{}", folder_path, file_name);

    writer_.open(file_path_, std::ios::out);
    if (!writer_.is_open()) {
      tools::logger()->error("[ProfileLog] failed to open profile log file: {}", file_path_);
      enabled_ = false;
      return;
    }

    writer_ << "profile_line\n";
    writer_.flush();
  }

  tools::logger()->info(
    "[ProfileLog] enabled={}, outputs={{stdout:{}, file:{}, ros2_topic:{}}}", enabled_,
    output_config_.stdout_enabled ? "on" : "off", output_config_.file_enabled ? "on" : "off",
    output_config_.ros2_topic_enabled ? "on" : "off");
  if (output_config_.file_enabled) {
    tools::logger()->info("[ProfileLog] writing profile log to {}", file_path_);
  }
}

ProfileLog::~ProfileLog()
{
  if (!enabled_) return;

  std::lock_guard<std::mutex> lk(mtx_);
  finalize_current_frame();
  flush_pending(true);
  if (writer_.is_open()) {
    writer_.flush();
    writer_.close();
  }
}

void ProfileLog::next_frame()
{
  if (!enabled_) return;
  std::lock_guard<std::mutex> lk(mtx_);

  const auto now = std::chrono::steady_clock::now();
  if (frame_started_) {
    last_frame_elapsed_ms_ =
      static_cast<double>(
        std::chrono::duration_cast<std::chrono::microseconds>(now - last_frame_time_).count()) /
      1000.0;
    finalize_current_frame();
    flush_if_needed();
  } else {
    last_frame_elapsed_ms_ = 0.0;
  }

  last_frame_time_ = now;
  ++frame_id_;
  frame_started_ = true;
  current_frame_records_.clear();
}

void ProfileLog::record(const std::string & module, double cost_ms)
{
  if (!enabled_) return;

  std::lock_guard<std::mutex> lk(mtx_);
  if (!frame_started_) {
    ++frame_id_;
    frame_started_ = true;
    current_frame_records_.clear();
  }
  current_frame_records_.emplace_back(module, cost_ms);
}

bool ProfileLog::enabled() const { return enabled_; }

void ProfileLog::flush_if_needed()
{
  if (pending_lines_ >= flush_every_) {
    flush_pending(false);
  }
}

void ProfileLog::flush_pending(bool force)
{
  if (!force && pending_lines_ < flush_every_) return;
  if (pending_frame_lines_.empty()) return;

  for (const auto & line : pending_frame_lines_) {
    if (output_config_.file_enabled && writer_.is_open()) {
      writer_ << line << "\n";
    }
    if (output_config_.stdout_enabled) {
      tools::logger()->info("[Profile] {}", line);
    }
    if (output_config_.ros2_topic_enabled && ros2_topic_publisher_) {
      ros2_topic_publisher_(line);
    }
  }

  if (output_config_.file_enabled && writer_.is_open()) {
    writer_.flush();
  }

  pending_frame_lines_.clear();
  pending_lines_ = 0;
}

void ProfileLog::finalize_current_frame()
{
  if (!frame_started_) return;
  if (current_frame_records_.empty()) return;

  double total_ms = 0.0;
  bool has_loop_total = false;
  std::string modules_text;

  for (std::size_t i = 0; i < current_frame_records_.size(); ++i) {
    const auto & record = current_frame_records_[i];
    const auto & module = record.first;
    const auto cost_ms = record.second;
    if (module == "loop.total") {
      total_ms = cost_ms;
      has_loop_total = true;
    }
  }

  if (!has_loop_total) {
    for (const auto & record : current_frame_records_) {
      total_ms += record.second;
    }
  }

  const double fps = last_frame_elapsed_ms_ > 0.0 ? 1000.0 / last_frame_elapsed_ms_ : 0.0;

  for (const auto & record : current_frame_records_) {
    const auto & module = record.first;
    const auto cost_ms = record.second;

    if (module == "loop.total") {
      continue;
    }

    modules_text += fmt::format("\t{}: {:.3f}ms\n", module, cost_ms);
  }

  const auto now = std::chrono::system_clock::now();
  const auto packed_line = fmt::format(
    "time: {:%H:%M:%S}\n"
    "frame: {}\n"
    "fps: {:.2f}\n"
    "total_ms: {:.3f}\n"
    "module_count: {}\n"
    "modules:\n"
    "{}",
    now, frame_id_, fps, total_ms, current_frame_records_.size(), modules_text);

  pending_frame_lines_.push_back(packed_line);
  pending_lines_ = pending_frame_lines_.size();

  current_frame_records_.clear();
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
