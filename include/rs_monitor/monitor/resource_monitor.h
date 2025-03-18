// Copyright 2025 RoboSense Technology Co., Ltd
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RS_MONITOR_MONITOR_RESOURCE_MONITOR_H
#define RS_MONITOR_MONITOR_RESOURCE_MONITOR_H

#include <chrono>
#include <set>
#include <unordered_map>

#include "rs_monitor/common/ros2_process_manager.h"
#include "rs_monitor/common/system_stats.h"
#include "rs_monitor/monitor/monitor_base.h"
#include "diagnostic_updater/diagnostic_updater.hpp"

namespace robosense::rs_monitor
{

/**
 * @struct ProcessUsageInfo
 * @brief Contains resource usage information for a ROS2 process
 *
 * Extends ROS2ProcessInfo with additional metrics for resource monitoring including
 * CPU, memory, and I/O usage statistics.
 */
struct ProcessUsageInfo : public ROS2ProcessInfo
{
  uint64_t prev_cpu_jiffies{0};     ///< Previous CPU usage measurement in jiffies
  uint64_t prev_io_read_bytes{0};   ///< Previous cumulative bytes read
  uint64_t prev_io_write_bytes{0};  ///< Previous cumulative bytes written

  float cpu_usage{0.0f};        ///< Current CPU usage percentage
  float io_read_mb_sec{0};      ///< Current I/O read rate in MB/s
  float io_write_mb_sec{0};     ///< Current I/O write rate in MB/s
  uint32_t memory_usage_kb{0};  ///< Current memory usage in KB

  ros_time_t prev_time{0};  ///< Timestamp of previous measurement

  /**
   * @brief Constructor that initializes from ROS2ProcessInfo
   * @param info Process information to initialize from
   */
  ProcessUsageInfo(ROS2ProcessInfo const & info) : ROS2ProcessInfo(info) {}

  /**
   * @brief Updates node names and timestamp from another process info
   * @param info Process information to update from
   */
  void update(ROS2ProcessInfo const & info)
  {
    node_names = info.node_names;
    last_update = info.last_update;
  }
};

/**
 * @class ResourceMonitor
 * @brief Monitors system and ROS2 process resource usage
 *
 * This class collects and reports CPU, memory, and I/O statistics for both
 * the overall system and individual ROS2 processes.
 */
class ResourceMonitor : public MonitorBase
{
public:
  /**
   * @brief Constructor for ResourceMonitor
   * @param nh Pointer to the generic node handle
   */
  explicit ResourceMonitor(NodeHandle * nh);

  /**
   * @brief Destructor
   */
  ~ResourceMonitor() override = default;

  /**
   * @brief Initializes the resource monitor
   *
   * Sets up the system statistics collector and configures diagnostic updaters
   * for both system-wide and per-process resource monitoring.
   *
   * @param config YAML configuration node
   * @return true if initialization succeeded, false otherwise
   */
  bool init(YAML::Node const & config) override;

protected:
  /**
   * @brief Periodic execution function (empty in this implementation)
   * @param current_timestamp_ms Current timestamp in milliseconds
   */
  void run_once(uint64_t current_timestamp_ms) override;

private:
  /**
   * @brief Updates diagnostic information with ROS2 node resource usage
   *
   * Collects and formats resource usage for all ROS2 processes including:
   * - CPU usage percentage
   * - Memory usage (MB and percentage)
   * - I/O read/write rates
   * - Associated ROS2 node names
   *
   * @param stat Diagnostic status wrapper to update
   */
  void update_node_resource_usage(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /**
   * @brief Updates diagnostic information with system-wide resource usage
   *
   * Collects and formats overall system statistics:
   * - Total CPU usage percentage
   * - Per-core CPU usage
   * - Memory usage (used/total)
   *
   * @param stat Diagnostic status wrapper to update
   */
  void update_system_resource_usage(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /**
   * @brief Updates resource usage statistics for a specific process
   *
   * Gathers the following metrics for the specified process:
   * - CPU usage percentage
   * - Memory usage in KB
   * - I/O read and write rates in MB/sec
   *
   * Platform-specific implementations exist for Windows and Linux.
   *
   * @param process Process usage information structure to update
   */
  void update_process_stats(ProcessUsageInfo & process);

  /**
   * @brief Formats memory usage information as a readable string
   *
   * @param used_kb Used memory in kilobytes
   * @param total_kb Total memory in kilobytes
   * @param percentage Memory usage percentage
   * @return Formatted memory usage string (e.g., "3.45GB / 16.00GB (21.56%)")
   */
  std::string format_memory_usage(uint32_t used_kb, uint32_t total_kb, float percentage);

private:
  SystemStats system_stats_;
  std::unique_ptr<SystemStatsCollector> system_stats_collector_;
  std::unordered_map<pid_t, ProcessUsageInfo> processes_{};
};

}  // namespace robosense::rs_monitor

#endif  // RS_MONITOR_MONITOR_RESOURCE_MONITOR_H
