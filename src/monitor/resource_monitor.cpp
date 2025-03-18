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

#include "rs_monitor/monitor/resource_monitor.h"

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <fstream>

#ifdef _WIN32
#include <psapi.h>
#include <windows.h>
#elif __linux__
#include <unistd.h>
#endif

#include <filesystem>

#include "rs_monitor/common/common.h"
#include "diagnostic_updater/diagnostic_updater.hpp"

namespace fs = std::filesystem;

namespace robosense::rs_monitor
{

ResourceMonitor::ResourceMonitor(NodeHandle * nh) : MonitorBase("resource_monitor", nh) {}

bool ResourceMonitor::init(YAML::Node const & config)
{
  if (!MonitorBase::init(config)) {
    return false;
  }

  system_stats_collector_ = SystemStatsCollector::create();
  if (!system_stats_collector_ || !system_stats_collector_->init()) {
    RS_ERROR(get_node(), "Failed to initialize system stats collector");
    return false;
  }
  system_stats_collector_->collect(system_stats_);

  auto & updater(get_updater());
  updater.setPeriod(std::round(get_exec_interval_ms() / 1e3));

  updater.add(
    "Node Resource Usage",
    std::bind(&ResourceMonitor::update_node_resource_usage, this, std::placeholders::_1));
  updater.add(
    "System Resource Usage",
    std::bind(&ResourceMonitor::update_system_resource_usage, this, std::placeholders::_1));

  return true;
}

void ResourceMonitor::run_once(uint64_t current_timestamp_ms)
{
  static_cast<void>(current_timestamp_ms);
#if __ROS1__
  auto ptr = &get_updater();
  if (ptr) {
    ptr->update();
  }
#endif
}

#ifdef _WIN32  // Windows implementation

void ResourceMonitor::update_process_stats(ProcessUsageInfo & info)
{
  HANDLE hProcess = OpenProcess(PROCESS_QUERY_INFORMATION | PROCESS_VM_READ, FALSE, info.pid);
  if (hProcess == NULL) {
    return;
  }

  auto current_time = SystemTime();
  auto time_diff = (current_time - info.prev_time) / 1e9;

  FILETIME createTime, exitTime, kernelTime, userTime;
  if (GetProcessTimes(hProcess, &createTime, &exitTime, &kernelTime, &userTime)) {
    uint64_t currentTime =
      (static_cast<uint64_t>(userTime.dwHighDateTime) << 32) | userTime.dwLowDateTime;
    currentTime +=
      (static_cast<uint64_t>(kernelTime.dwHighDateTime) << 32) | kernelTime.dwLowDateTime;

    if (info.prev_time > 0 && time_diff > 0) {
      info.cpu_usage = ((currentTime - info.prev_cpu_jiffies) / (time_diff * 10000.0)) * 100.0;
      if (info.cpu_usage > 10000.0 || info.cpu_usage < 0.0) {
        RS_WARN(
          get_node(), "Abnormal CPU usage [%f] detected for process %d", info.cpu_usage, info.pid);
        info.cpu_usage = nanf("0");
      }
    } else {
      // 第一次采集，初始化时间点但不计算CPU使用率
      info.cpu_usage = nanf("0");
    }

    info.prev_cpu_jiffies = currentTime;
  }

  // memory
  PROCESS_MEMORY_COUNTERS pmc;
  if (GetProcessMemoryInfo(hProcess, &pmc, sizeof(pmc))) {
    info.memory_usage_kb = pmc.WorkingSetSize / 1024;
  }

  // io
  IO_COUNTERS ioCounters;
  if (GetProcessIoCounters(hProcess, &ioCounters)) {
    uint64_t readBytes = ioCounters.ReadTransferCount;
    uint64_t writeBytes = ioCounters.WriteTransferCount;

    if (info.prev_time.seconds() > 0 && time_diff > 0) {
      info.io_read_mb_sec = ((readBytes - info.prev_io_read_bytes) / 1048576.f) / time_diff;
      info.io_write_mb_sec = ((writeBytes - info.prev_io_write_bytes) / 1048576.f) / time_diff;
    } else {
      info.io_read_mb_sec = nanf("0");
      info.io_write_mb_sec = nanf("0");
    }

    info.prev_io_read_bytes = readBytes;
    info.prev_io_write_bytes = writeBytes;
  }

  info.prev_time = current_time;

  CloseHandle(hProcess);
}

#else  // Linux implementation

void ResourceMonitor::update_process_stats(ProcessUsageInfo & info)
{
  std::string stat_path = "/proc/" + std::to_string(info.pid) + "/stat";
  std::ifstream stat_file(stat_path);
  if (!stat_file) {
    return;
  }

  std::string line{};
  std::getline(stat_file, line);
  std::istringstream iss(line);

  std::string unused{};
  uint64_t utime = 0, stime = 0, cutime = 0, cstime = 0;
  for (int i = 1; i <= 13; ++i) {
    iss >> unused;
  }
  iss >> utime >> stime >> cutime >> cstime;

  uint64_t jiffies = utime + stime + cutime + cstime;
  auto current_time = SystemTime();
  auto time_diff = (current_time - info.prev_time) / 1e9;

  if (info.prev_time > 0 && time_diff > 0) {
    info.cpu_usage =
      ((jiffies - info.prev_cpu_jiffies) / (time_diff * sysconf(_SC_CLK_TCK))) * 100.0;
    if (info.cpu_usage > 10000.0 || info.cpu_usage < 0.0) {
      RS_WARN(
        get_node(), "Abnormal CPU usage [%f] detected for process %d", info.cpu_usage, info.pid);
      info.cpu_usage = nanf("0");
    }
  } else {
    // 第一次采集，初始化时间点但不计算CPU使用率
    info.cpu_usage = nanf("0");
  }

  info.prev_cpu_jiffies = jiffies;

  // memory
  std::string status_path = "/proc/" + std::to_string(info.pid) + "/status";
  std::ifstream status_file(status_path);
  if (!status_file) {
    return;
  }

  while (std::getline(status_file, line)) {
    if (line.find("VmRSS:") == 0) {
      info.memory_usage_kb = parse_uint64_in_string(line);
      break;
    }
  }

  // io
  uint64_t read_bytes = 0, write_bytes = 0;

  std::string io_path = "/proc/" + std::to_string(info.pid) + "/io";
  std::ifstream io_file(io_path);
  if (!io_file) {
    return;
  }

  while (std::getline(io_file, line)) {
    if (line.find("read_bytes:") == 0) {
      read_bytes = parse_uint64_in_string(line);
    } else if (line.find("write_bytes:") == 0) {
      write_bytes = parse_uint64_in_string(line);
    }
  }

  if (info.prev_time > 0 && time_diff > 0) {
    info.io_read_mb_sec = ((read_bytes - info.prev_io_read_bytes) / 1048576.f) / time_diff;
    info.io_write_mb_sec = ((write_bytes - info.prev_io_write_bytes) / 1048576.f) / time_diff;
  } else {
    info.io_read_mb_sec = nanf("0");
    info.io_write_mb_sec = nanf("0");
  }

  info.prev_io_read_bytes = read_bytes;
  info.prev_io_write_bytes = write_bytes;

  info.prev_time = current_time;
}

#endif

void ResourceMonitor::update_system_resource_usage(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (!system_stats_collector_->collect(system_stats_)) {
    stat.summary(2, "Failed to collect system stats");  // ERROR
    return;
  }

  stat.summary(0, "System resource usage");  // OK

  char buffer[64]{0};
  sprintf(buffer, "%.2f%%", system_stats_.total_cpu_usage);

  stat.add("Total CPU Usage", buffer);
  stat.add(
    "Memory Usage",
    format_memory_usage(
      system_stats_.used_memory_kb, system_stats_.total_memory_kb, system_stats_.memory_usage));

  for (size_t i = 0; i < system_stats_.core_usage.size(); ++i) {
    sprintf(buffer, "%.2f%%", system_stats_.core_usage[i]);
    stat.add("Processor " + std::to_string(i), buffer);
  }
}

void ResourceMonitor::update_node_resource_usage(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  auto ros_processes(ROS2ProcessManager::instance().get_processes());

  // 更新统计值
  for (auto const & [pid, info] : ros_processes) {
    auto it = processes_.find(pid);
    if (it == processes_.end()) {
      processes_.emplace(std::make_pair(pid, ProcessUsageInfo(info)));
    } else {
      it->second.update(info);
    }
  }

  stat.summary(DiagnosticStatus::OK, "ROS2 Process Resource Usage");

  char buffer[256]{0};

  // 添加详细进程信息, 剔除不存在的进程
  for (auto it = processes_.begin(); it != processes_.end();) {
    auto ros_it = ros_processes.find(it->first);
    if (ros_it == ros_processes.end()) {
      auto temp = it++;
      processes_.erase(temp);
    } else {
      update_process_stats(it->second);
      if (it->second.name.empty()) {
        continue;
      }

      std::string nodes{};
      for (auto const & node : it->second.node_names) {
        nodes.append("<").append(node).append(">");
      }

      sprintf(
        buffer,
        "CPU: %.2f%% | Memory: %.1fMB (%.2f%%) | PID: %d | Read: (%.3fMB/s) | Write: (%.3fMB/s) | "
        "Nodes: %s",
        it->second.cpu_usage, it->second.memory_usage_kb / 1024.0f,
        it->second.memory_usage_kb / static_cast<float>(system_stats_.total_memory_kb) * 100.f,
        it->second.pid, it->second.io_read_mb_sec, it->second.io_write_mb_sec, nodes.c_str());

      stat.add(it->second.name, buffer);

      ++it;
    }
  }
}

std::string ResourceMonitor::format_memory_usage(
  uint32_t used_kb, uint32_t total_kb, float percentage)
{
  char result[64]{0};
  std::snprintf(
    result, sizeof(result), "%.2fGB / %.3fGB (%.2f%%)", used_kb / 1048576.f, total_kb / 1048576.f,
    percentage);

  return std::string(result);
}

}  // namespace robosense::rs_monitor
