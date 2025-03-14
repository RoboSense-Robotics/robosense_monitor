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
