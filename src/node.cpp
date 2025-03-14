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
