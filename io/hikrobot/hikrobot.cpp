#include "hikrobot.hpp"

#include <libusb-1.0/libusb.h>

#include <algorithm>
#include <cstring>
#include <unordered_map>

#include "tools/logger.hpp"

using namespace std::chrono_literals;

namespace io
{
namespace
{
constexpr unsigned int kMvNoData = 0x80000007;  // MV_E_NODATA

std::string to_string_fixed(const unsigned char * chars, size_t max_len)
{
  size_t len = 0;
  while (len < max_len && chars[len] != '\0') ++len;
  return std::string(reinterpret_cast<const char *>(chars), len);
}
}  // namespace

HikRobot::HikRobot(
  double exposure_ms, double gain, const std::string & vid_pid, const std::string & serial_number)
: exposure_us_(exposure_ms * 1e3),
  gain_(gain),
  daemon_quit_(false),
  handle_(nullptr),
  capturing_(false),
  capture_quit_(false),
  queue_(1),
  has_last_data_(false),
  vid_(-1),
  pid_(-1),
  serial_number_(serial_number)
{
  set_vid_pid(vid_pid);
  if (libusb_init(NULL)) tools::logger()->warn("Unable to init libusb!");

  daemon_thread_ = std::thread{[this] {
    tools::logger()->info("HikRobot's daemon thread started.");

    capture_start();

    auto retry_sleep = 200ms;
    int recover_fail_count = 0;

    while (!daemon_quit_) {
      std::this_thread::sleep_for(retry_sleep);

      if (capturing_) {
        retry_sleep = 200ms;
        recover_fail_count = 0;
        continue;
      }

      capture_stop();

      // USB reset is expensive; only do it occasionally when repeated recovery fails.
      if (recover_fail_count % 3 == 2) {
        reset_usb();
      }

      capture_start();

      if (!capturing_) {
        recover_fail_count++;
        retry_sleep = std::min(retry_sleep * 2, 2000ms);
      } else {
        retry_sleep = 200ms;
      }
    }

    capture_stop();

    tools::logger()->info("HikRobot's daemon thread stopped.");
  }};
}

HikRobot::~HikRobot()
{
  daemon_quit_ = true;
  if (daemon_thread_.joinable()) daemon_thread_.join();
  tools::logger()->info("HikRobot destructed.");
}

void HikRobot::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  CameraData data;
  if (!queue_.pop_for(data, 50ms)) {
    if (!has_last_data_) {
      static auto last_warn = std::chrono::steady_clock::time_point::min();
      auto now = std::chrono::steady_clock::now();
      if (
        last_warn == std::chrono::steady_clock::time_point::min() ||
        std::chrono::duration_cast<std::chrono::seconds>(now - last_warn).count() >= 1) {
        tools::logger()->warn("HikRobot read timeout: no frame available yet");
        last_warn = now;
      }
      img = cv::Mat();
      timestamp = now;
      return;
    }
    data = last_data_;
  } else {
    last_data_ = data;
    has_last_data_ = true;
  }

  img = data.img;
  timestamp = data.timestamp;
}

void HikRobot::capture_start()
{
  handle_ = nullptr;
  capturing_ = false;
  capture_quit_ = false;

  unsigned int ret;

  MV_CC_DEVICE_INFO_LIST device_list;
  ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_EnumDevices failed: {:#x}", ret);
    return;
  }

  if (device_list.nDeviceNum == 0) {
    tools::logger()->warn("Not found camera!");
    return;
  }

  auto * selected_device = select_device(device_list);
  if (!selected_device) {
    tools::logger()->warn("No HikRobot device matched serial_number: \"{}\"", serial_number_);
    return;
  }

  ret = MV_CC_CreateHandle(&handle_, selected_device);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_CreateHandle failed: {:#x}", ret);
    handle_ = nullptr;
    return;
  }

  ret = MV_CC_OpenDevice(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_OpenDevice failed: {:#x}", ret);
    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
    return;
  }

  set_enum_value("AcquisitionMode", MV_ACQ_MODE_CONTINUOUS);
  set_enum_value("TriggerMode", MV_TRIGGER_MODE_OFF);
  set_enum_value("BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
  set_enum_value("ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
  set_enum_value("GainAuto", MV_GAIN_MODE_OFF);
  set_float_value("ExposureTime", exposure_us_);
  set_float_value("Gain", gain_);
  MV_CC_SetFrameRate(handle_, 150);

  ret = MV_CC_StartGrabbing(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_StartGrabbing failed: {:#x}", ret);
    MV_CC_CloseDevice(handle_);
    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
    return;
  }

  // Mark as active before thread launch to avoid daemon race on startup.
  capturing_ = true;

  capture_thread_ = std::thread{[this] {
    tools::logger()->info("HikRobot's capture thread started.");
    int timeout_count = 0;
    auto last_timeout_log = std::chrono::steady_clock::time_point::min();

    MV_FRAME_OUT raw;
    MV_CC_PIXEL_CONVERT_PARAM cvt_param;

    while (!capture_quit_) {
      std::this_thread::sleep_for(1ms);

      unsigned int ret;
      unsigned int nMsec = 100;

      ret = MV_CC_GetImageBuffer(handle_, &raw, nMsec);
      if (ret != MV_OK) {
        if (ret == kMvNoData) {
          timeout_count++;
          auto now = std::chrono::steady_clock::now();
          if (
            last_timeout_log == std::chrono::steady_clock::time_point::min() ||
            std::chrono::duration_cast<std::chrono::seconds>(now - last_timeout_log).count() >= 1) {
            tools::logger()->warn(
              "MV_CC_GetImageBuffer timeout: {:#x} ({} consecutive)", ret, timeout_count);
            last_timeout_log = now;
          }

          // Tolerate transient no-data periods to avoid endless reconnect churn.
          if (timeout_count < 15) {
            continue;
          }
        } else {
          tools::logger()->warn("MV_CC_GetImageBuffer failed: {:#x}", ret);
        }
        break;
      }
      timeout_count = 0;

      auto timestamp = std::chrono::steady_clock::now();
      cv::Mat img(cv::Size(raw.stFrameInfo.nWidth, raw.stFrameInfo.nHeight), CV_8U, raw.pBufAddr);

      cvt_param.nWidth = raw.stFrameInfo.nWidth;
      cvt_param.nHeight = raw.stFrameInfo.nHeight;

      cvt_param.pSrcData = raw.pBufAddr;
      cvt_param.nSrcDataLen = raw.stFrameInfo.nFrameLen;
      cvt_param.enSrcPixelType = raw.stFrameInfo.enPixelType;

      cvt_param.pDstBuffer = img.data;
      cvt_param.nDstBufferSize = img.total() * img.elemSize();
      cvt_param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;

      // ret = MV_CC_ConvertPixelType(handle_, &cvt_param);
      const auto & frame_info = raw.stFrameInfo;
      auto pixel_type = frame_info.enPixelType;
      cv::Mat dst_image;
      const static std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes> type_map = {
        {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2RGB},
        {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB},
        {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2RGB},
        {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2RGB}};
      cv::cvtColor(img, dst_image, type_map.at(pixel_type));
      img = dst_image;

      queue_.push({img, timestamp});

      ret = MV_CC_FreeImageBuffer(handle_, &raw);
      if (ret != MV_OK) {
        tools::logger()->warn("MV_CC_FreeImageBuffer failed: {:#x}", ret);
        break;
      }
    }

    capturing_ = false;
    tools::logger()->info("HikRobot's capture thread stopped.");
  }};
}

void HikRobot::capture_stop()
{
  capture_quit_ = true;
  if (capture_thread_.joinable()) capture_thread_.join();

  if (!handle_) return;

  unsigned int ret;

  ret = MV_CC_StopGrabbing(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_StopGrabbing failed: {:#x}", ret);
    return;
  }

  ret = MV_CC_CloseDevice(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_CloseDevice failed: {:#x}", ret);
    return;
  }

  ret = MV_CC_DestroyHandle(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_DestroyHandle failed: {:#x}", ret);
    return;
  }

  handle_ = nullptr;
}

MV_CC_DEVICE_INFO * HikRobot::select_device(const MV_CC_DEVICE_INFO_LIST & device_list) const
{
  MV_CC_DEVICE_INFO * fallback = nullptr;

  for (unsigned int i = 0; i < device_list.nDeviceNum; ++i) {
    auto * device = device_list.pDeviceInfo[i];
    if (!device) continue;

    if ((device->nTLayerType & MV_USB_DEVICE) == 0) continue;

    auto serial =
      to_string_fixed(device->SpecialInfo.stUsb3VInfo.chSerialNumber, INFO_MAX_BUFFER_SIZE);
    auto model = to_string_fixed(device->SpecialInfo.stUsb3VInfo.chModelName, INFO_MAX_BUFFER_SIZE);
    tools::logger()->info("HikRobot device[{}]: model={}, serial={}", i, model, serial);

    if (!fallback) fallback = device;
    if (!serial_number_.empty() && serial == serial_number_) return device;
  }

  if (!serial_number_.empty()) return nullptr;
  return fallback;
}

void HikRobot::set_float_value(const std::string & name, double value)
{
  unsigned int ret;

  ret = MV_CC_SetFloatValue(handle_, name.c_str(), value);

  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_SetFloatValue(\"{}\", {}) failed: {:#x}", name, value, ret);
    return;
  }
}

void HikRobot::set_enum_value(const std::string & name, unsigned int value)
{
  unsigned int ret;

  ret = MV_CC_SetEnumValue(handle_, name.c_str(), value);

  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_SetEnumValue(\"{}\", {}) failed: {:#x}", name, value, ret);
    return;
  }
}

void HikRobot::set_vid_pid(const std::string & vid_pid)
{
  auto index = vid_pid.find(':');
  if (index == std::string::npos) {
    tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
    return;
  }

  auto vid_str = vid_pid.substr(0, index);
  auto pid_str = vid_pid.substr(index + 1);

  try {
    vid_ = std::stoi(vid_str, 0, 16);
    pid_ = std::stoi(pid_str, 0, 16);
  } catch (const std::exception &) {
    tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
  }
}

void HikRobot::reset_usb() const
{
  if (vid_ == -1 || pid_ == -1) return;

  // https://github.com/ralight/usb-reset/blob/master/usb-reset.c
  auto handle = libusb_open_device_with_vid_pid(NULL, vid_, pid_);
  if (!handle) {
    tools::logger()->warn("Unable to open usb!");
    return;
  }

  if (libusb_reset_device(handle))
    tools::logger()->warn("Unable to reset usb!");
  else
    tools::logger()->info("Reset usb successfully :)");

  libusb_close(handle);
}

}  // namespace io