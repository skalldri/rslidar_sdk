/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#pragma once

#ifdef ROS2_FOUND

#include "source/source.hpp"

#include <rs_driver/api/lidar_driver.hpp>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace robosense
{
namespace lidar
{

// The LiDAR's health on /diagnostics (diagnostic_msgs/DiagnosticArray), once a second, through
// diagnostic_updater, for diagnostic_aggregator and rqt_robot_monitor.
//
// One status, named "<node_name>: LiDAR". Its values:
// - the temperature the LiDAR reports in its MSOP packets (LidarDriver::getTemperature), while
//   packets arrive;
// - the point cloud rate, and the IMU rate when an IMU is expected or has sent data;
// - the driver's packet timeouts and packet error reports since start, and the last error;
// - the model, and the serial number and firmware versions from the DIFOP packets.
//
// Levels: ERROR when no point cloud came in the last period, or at or above temperature_error.
// WARN at or above temperature_warn, on a packet error in the last HOLD_SECONDS, or when an
// expected IMU sends nothing. A threshold of 0 turns that check off.
//
// rs_driver reports most packet errors at most once every 2 s (LIMIT_CALL), and its MSOP timeout
// once per 1 s select() timeout, so a 1 s period can miss either while the fault persists. The
// status holds both for HOLD_SECONDS instead of counting them per period.
class SourceDiagnostics
{
public:
  SourceDiagnostics(const std::string& node_name, const std::string& lidar_type, bool imu_expected,
                    float temperature_warn, float temperature_error,
                    std::shared_ptr<LidarDriver<LidarPointCloudMsg>> driver);
  ~SourceDiagnostics();

  // Called from the driver's threads.
  void countPointCloud()
  {
    ++point_clouds_;
  }
  void countImuData()
  {
    ++imu_data_;
  }
  void countError(const Error& error);

private:
  static constexpr double HOLD_SECONDS = 3.0;

  void update(diagnostic_updater::DiagnosticStatusWrapper& stat);
  static std::string toHex(const uint8_t* data, size_t size);
  static int64_t nowNs()
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
  }

  std::string lidar_type_;
  float temperature_warn_;
  float temperature_error_;
  std::shared_ptr<LidarDriver<LidarPointCloudMsg>> driver_;

  std::atomic<uint32_t> point_clouds_{ 0 };
  std::atomic<uint32_t> imu_data_{ 0 };
  std::atomic<uint64_t> timeouts_total_{ 0 };
  std::atomic<uint64_t> errors_total_{ 0 };
  // steady_clock, ns; 0 = never
  std::atomic<int64_t> last_timeout_ns_{ 0 };
  std::atomic<int64_t> last_error_ns_{ 0 };
  std::mutex last_error_mutex_;
  std::string last_error_;
  int64_t last_error_report_ns_{ 0 };  // of last_error_, under last_error_mutex_

  bool imu_shown_;  // an IMU is expected, or has sent data
  int64_t last_update_ns_;

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<diagnostic_updater::Updater> updater_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
};

inline SourceDiagnostics::SourceDiagnostics(const std::string& node_name, const std::string& lidar_type,
                                            bool imu_expected, float temperature_warn, float temperature_error,
                                            std::shared_ptr<LidarDriver<LidarPointCloudMsg>> driver)
  : lidar_type_(lidar_type)
  , temperature_warn_(temperature_warn)
  , temperature_error_(temperature_error)
  , driver_(driver)
  , imu_shown_(imu_expected)
  , last_update_ns_(nowNs())
{
  // A node-local __node remap, so that a process-wide one (launch_ros's Node(name=...)) doesn't
  // rename this node too.
  node_ = rclcpp::Node::make_shared(
      node_name, rclcpp::NodeOptions().arguments({ "--ros-args", "-r", "__node:=" + node_name }));
  updater_.reset(new diagnostic_updater::Updater(node_, 1.0));
  updater_->setHardwareID("RoboSense " + lidar_type_);
  updater_->add("LiDAR", this, &SourceDiagnostics::update);

  // The Updater publishes from a timer on its node; this node has an executor of its own.
  executor_.add_node(node_);
  spin_thread_ = std::thread([this]() { executor_.spin(); });
}

inline SourceDiagnostics::~SourceDiagnostics()
{
  executor_.cancel();
  if (spin_thread_.joinable())
  {
    spin_thread_.join();
  }
  updater_.reset();
}

inline void SourceDiagnostics::countError(const Error& error)
{
  if (error.error_code_type == ErrCodeType::INFO_CODE)  // the pcap's end, a repeat
  {
    return;
  }
  const int64_t now = nowNs();
  if (error.error_code == ERRCODE_MSOPTIMEOUT)  // no packet for 1 s
  {
    ++timeouts_total_;
    last_timeout_ns_ = now;
  }
  else
  {
    ++errors_total_;
    last_error_ns_ = now;
  }

  std::lock_guard<std::mutex> lock(last_error_mutex_);
  last_error_ = error.toString();
  last_error_report_ns_ = now;
}

inline std::string SourceDiagnostics::toHex(const uint8_t* data, size_t size)
{
  std::string hex;
  char byte[3];
  for (size_t i = 0; i < size; i++)
  {
    snprintf(byte, sizeof(byte), "%02X", data[i]);
    hex += byte;
  }
  return hex;
}

inline void SourceDiagnostics::update(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  const int64_t now = nowNs();
  const double period = (now - last_update_ns_) * 1e-9;
  last_update_ns_ = now;
  auto recent = [now](int64_t t) { return t != 0 && (now - t) * 1e-9 < HOLD_SECONDS; };

  const uint32_t point_clouds = point_clouds_.exchange(0);
  const uint32_t imu_data = imu_data_.exchange(0);
  imu_shown_ = imu_shown_ || imu_data > 0;

  std::string last_error = "none";
  {
    std::lock_guard<std::mutex> lock(last_error_mutex_);
    if (!last_error_.empty())
    {
      char age[32];
      snprintf(age, sizeof(age), ", %.0f s ago", (now - last_error_report_ns_) * 1e-9);
      last_error = last_error_ + age;
    }
  }

  char value[32];

  if (point_clouds == 0)
  {
    stat.summary(DiagnosticStatus::ERROR, recent(last_timeout_ns_.load()) ? "No packets from the LiDAR" :
                                                                            "No point clouds from the LiDAR");
    stat.add("Temperature (°C)", "no data");
  }
  else
  {
    stat.summary(DiagnosticStatus::OK, "Running");

    // Updated by every MSOP packet; the point clouds show that packets came in this period.
    float temperature;
    if (driver_->getTemperature(temperature))
    {
      snprintf(value, sizeof(value), "%.1f", temperature);
      stat.add("Temperature (°C)", value);
      if (temperature_error_ > 0 && temperature >= temperature_error_)
      {
        stat.mergeSummary(DiagnosticStatus::ERROR, "Temperature high");
      }
      else if (temperature_warn_ > 0 && temperature >= temperature_warn_)
      {
        stat.mergeSummary(DiagnosticStatus::WARN, "Temperature high");
      }
    }
    else
    {
      stat.add("Temperature (°C)", "not reported");
    }

    if (recent(last_error_ns_.load()))
    {
      stat.mergeSummary(DiagnosticStatus::WARN, "Packet errors");
    }
    if (imu_shown_ && imu_data == 0)
    {
      stat.mergeSummary(DiagnosticStatus::WARN, "No IMU data");
    }
  }

  snprintf(value, sizeof(value), "%.1f", period > 0 ? point_clouds / period : 0.0);
  stat.add("Point clouds (Hz)", value);
  if (imu_shown_)
  {
    snprintf(value, sizeof(value), "%.1f", period > 0 ? imu_data / period : 0.0);
    stat.add("IMU (Hz)", value);
  }
  stat.add("Packet timeouts (total)", std::to_string(timeouts_total_.load()));
  // rs_driver reports each kind of packet error at most once every 2 s, so this counts reports.
  stat.add("Packet error reports (total)", std::to_string(errors_total_.load()));
  stat.add("Last packet error", last_error);
  stat.add("Model", lidar_type_);

  DeviceInfo info;
  if (driver_->getDeviceInfo(info))
  {
    const std::string serial = toHex(info.sn, sizeof(info.sn));
    stat.add("Serial number", serial);
    stat.add("Firmware (top board)", toHex(info.top_ver, sizeof(info.top_ver)));
    stat.add("Firmware (bottom board)", toHex(info.bottom_ver, sizeof(info.bottom_ver)));
    updater_->setHardwareID("RoboSense " + lidar_type_ + " " + serial);
  }
}

}  // namespace lidar
}  // namespace robosense

#endif  // ROS2_FOUND
