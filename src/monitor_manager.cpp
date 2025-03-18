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

#include "rs_monitor/monitor_manager.h"

#include <filesystem>
#include <chrono>

#include "rs_monitor/common/common.h"
#include "rs_monitor/common/ros2_process_manager.h"
#include "rs_monitor/monitor/frequency_monitor.h"
#include "rs_monitor/monitor/resource_monitor.h"
#include "rs_monitor/monitor/timestamp_monitor.h"

namespace fs = std::filesystem;

namespace robosense::rs_monitor
{

// this will be overwriten by launch.py
char const * kDefaultNodeName = "rs_monitor";

template <class MonitorType, class... MonitorTypes>
void append_monitors(
  std::vector<std::shared_ptr<MonitorBase>> & monitors, YAML::Node const & config, NodeHandle & nh)
{
  auto monitor(std::make_shared<MonitorType>(&nh));
  bool is_initialized = monitor->init(config);
  if (is_initialized) {
    monitors.emplace_back(std::move(monitor));
  }

  if constexpr (sizeof...(MonitorTypes) > 0) {
    append_monitors<MonitorTypes...>(monitors, config, nh);
  }
}

#if __ROS2__
MonitorManager::MonitorManager(rclcpp::NodeOptions const & options)
: Node(kDefaultNodeName, options)
#elif __ROS1__
MonitorManager::MonitorManager(NodeHandle & nh)
#endif
{
#if __ROS2__
  auto & nh_ref_ = *this;
#elif __ROS1__
  auto & nh_ref_ = nh;
#endif

  std::string config_file_path{};
#if __ROS2__
  declare_parameter<std::string>("config_path", "");
  config_file_path = get_parameter("config_path").as_string();
#elif __ROS1__
  nh.getParam("config_path", config_file_path);
#endif

  YAML::Node config;
  if (config_file_path.empty()) {
    // load default config from package resources
    config = load_config<true>(nh_ref_, "config/config.yaml");
  } else {
    config = load_config<true, true>(nh_ref_, config_file_path);
  }

  if (!ROS2ProcessManager::instance().init(config, nh_ref_)) {
    throw std::runtime_error("Failed to initialize ROS2ProcessManager");
  }

  append_monitors<ResourceMonitor, FrequencyMonitor, TimestampMonitor>(monitors_, config, nh_ref_);

  timer_ =
#if __ROS2__
    create_wall_timer(std::chrono::milliseconds(200), std::bind(&MonitorManager::run_once, this));
#elif __ROS1__
    nh.createWallTimer(
      ros::WallDuration(0.2), [this](ros::WallTimerEvent const &) { this->run_once(); });
#endif
}

void MonitorManager::run_once()
{
  for (auto & monitor_ptr : monitors_) {
    uint64_t current_time_ms = SystemTime() / 1e6;
    monitor_ptr->update(current_time_ms);
  }
}

}  // namespace robosense::rs_monitor

#if __ROS2__
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robosense::rs_monitor::MonitorManager)
#endif
