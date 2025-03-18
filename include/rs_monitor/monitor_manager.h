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

#ifndef RS_MONITOR_MONITOR_MANAGER_H
#define RS_MONITOR_MONITOR_MANAGER_H

#include <memory>

#include "rs_monitor/common/ros_adapter.h"

namespace robosense::rs_monitor
{

class MonitorBase;

#if __ROS2__
class MonitorManager : public rclcpp::Node
#elif __ROS1__
class MonitorManager
#endif
{
public:
#if __ROS2__
  explicit MonitorManager(rclcpp::NodeOptions const & options);
#elif __ROS1__
  explicit MonitorManager(ros::NodeHandle & nh);
#endif
  virtual ~MonitorManager() = default;

protected:
  void run_once();

private:
  ROSTimer timer_;
  std::vector<std::shared_ptr<MonitorBase>> monitors_;
};

}  // namespace robosense::rs_monitor

#endif  // RS_MONITOR_MONITOR_MANAGER_H
