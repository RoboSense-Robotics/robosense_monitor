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

#ifndef RS_MONITOR_COMMON_SYSTEM_STATS_H
#define RS_MONITOR_COMMON_SYSTEM_STATS_H

#include <memory>
#include <string>
#include <vector>

namespace robosense::rs_monitor
{

struct SystemStats
{
  float total_cpu_usage{0.0f};
  std::vector<double> core_usage;
  uint32_t total_memory_kb{0};  // unit: KB
  uint32_t used_memory_kb{0};   // unit: KB
  float memory_usage{0.0f};
};

class SystemStatsCollector
{
public:
  virtual ~SystemStatsCollector() = default;
  virtual bool init() = 0;
  virtual bool collect(SystemStats & stats) = 0;

  static std::unique_ptr<SystemStatsCollector> create();
};

}  // namespace robosense::rs_monitor

#endif  // RS_MONITOR_COMMON_SYSTEM_STATS_H
