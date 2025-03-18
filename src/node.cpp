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

#if __ROS1__

#include <string>

#include "rs_monitor/common/ros_adapter.h"
#include "rs_monitor/monitor_manager.h"

int main(int argc, char * argv[])
{
  std::string default_node_name = "rs_monitor";
  ros::init(argc, argv, default_node_name);

  ros::NodeHandle nh;

  robosense::rs_monitor::MonitorManager monitor_manager(nh);
  ROS_INFO("ROS Monitor node '%s' started", default_node_name.c_str());

  ros::spin();

  return 0;
}

#endif
