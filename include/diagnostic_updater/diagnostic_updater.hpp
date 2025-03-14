#ifndef DIAGNOSTIC_UPDATER_WRAPPER_H
#define DIAGNOSTIC_UPDATER_WRAPPER_H

#if __ROS1__
#include "diagnostic_updater/ros1/diagnostic_updater.h"
#elif __ROS2__
#include "diagnostic_updater/ros2/diagnostic_updater.hpp"
#endif

namespace robosense::rs_monitor
{

#if __ROS2__
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
#elif __ROS1__
using DiagnosticStatus = diagnostic_msgs::DiagnosticStatus;
#endif

}  // namespace robosense::rs_monitor

#endif  // DIAGNOSTIC_UPDATER_WRAPPER_H
