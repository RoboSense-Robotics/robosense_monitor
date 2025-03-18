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

#ifndef RS_MONITOR_ROS_ADAPTER_H
#define RS_MONITOR_ROS_ADAPTER_H

#include <memory>
#include <type_traits>
#include <utility>
#include <sstream>
#include <iostream>

#if __ROS2__
#include <rclcpp/rclcpp.hpp>

#define ROSTime rclcpp::Clock(RCL_ROS_TIME).now().nanoseconds
#define SystemTime rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds

using NodeHandle = rclcpp::Node;
using ros_time_t = int64_t;
using ROSTimer = rclcpp::TimerBase::SharedPtr;
using GenericSubscriber = rclcpp::GenericSubscription::SharedPtr;
using SerializedMessage = rclcpp::SerializedMessage;
#if RCLCPP_VERSION_MAJOR > 16
#define CALLBACK_PARAM_TYPE(T) std::shared_ptr<T const>  // or usual ptr?
#else
#define CALLBACK_PARAM_TYPE(T) std::shared_ptr<T const>
#endif

#elif __ROS1__
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <topic_tools/shape_shifter.h>
#include <ros/master.h>

#define ROSTime ros::Time::now().toNSec
#define SystemTime ros::Time::now().toNSec

using NodeHandle = ros::NodeHandle;
using ros_time_t = uint64_t;
using ROSTimer = ros::WallTimer;
using GenericSubscriber = boost::shared_ptr<ros::Subscriber>;
using SerializedMessage = topic_tools::ShapeShifter;

#define CALLBACK_PARAM_TYPE(T) boost::shared_ptr<T const>

#endif

// common interface
using NodeHandlePtr = std::shared_ptr<NodeHandle>;
using GenericSubscriberMap = std::map<std::string, GenericSubscriber>;

/************************* LOG ADAPTER *************************/
namespace robosense::rs_monitor
{
template <typename T, typename = void>
struct LoggerGetter
{
  // Default implementation, will trigger compile error if no specialization matches
  static_assert(sizeof(T) == 0, "Unsupported logger type");
};

#if __ROS2__

template <>
struct LoggerGetter<rclcpp::Logger>
{
  static auto get(const rclcpp::Logger & logger) { return logger; }
};
template <>
struct LoggerGetter<rclcpp::Node>
{
  static auto get(const rclcpp::Node & node) { return node.get_logger(); }
};
template <>
struct LoggerGetter<rclcpp::Node *>
{
  static auto get(rclcpp::Node * node) { return node->get_logger(); }
};
template <typename T>
struct LoggerGetter<::std::shared_ptr<T>, std::enable_if_t<std::is_base_of_v<rclcpp::Node, T>>>
{
  static auto get(const ::std::shared_ptr<T> & node) { return node->get_logger(); }
};

#define DECLARE_LOGGER(__LEVEL__)                                                  \
  template <typename LoggerT, typename... Args>                                    \
  void RS_##__LEVEL__(LoggerT && logger_source, const char * fmt, Args &&... args) \
  {                                                                                \
    auto logger = robosense::rs_monitor::LoggerGetter<std::decay_t<LoggerT>>::get( \
      std::forward<LoggerT>(logger_source));                                       \
    RCLCPP_##__LEVEL__(logger, fmt, std::forward<Args>(args)...);                  \
  }                                                                                \
                                                                                   \
  template <typename LoggerT, typename T>                                          \
  void RS_##__LEVEL__##_STREAM(LoggerT && logger_source, T && msg)                 \
  {                                                                                \
    auto logger = robosense::rs_monitor::LoggerGetter<std::decay_t<LoggerT>>::get( \
      std::forward<LoggerT>(logger_source));                                       \
    RCLCPP_##__LEVEL__##_STREAM(logger, std::forward<T>(msg));                     \
  }

#elif __ROS1__

// ROS1: ros::NodeHandle (by reference)
template <>
struct LoggerGetter<ros::NodeHandle>
{
  static auto get(const ros::NodeHandle &) { return 0; }  // Dummy value, ROS1 doesn't need a logger
};

// ROS1: ros::NodeHandle* (pointer)
template <>
struct LoggerGetter<ros::NodeHandle *>
{
  static auto get(ros::NodeHandle *) { return 0; }
};

// ROS1: std::shared_ptr<ros::NodeHandle>
template <>
struct LoggerGetter<::std::shared_ptr<ros::NodeHandle>>
{
  static auto get(const ::std::shared_ptr<ros::NodeHandle> &) { return 0; }
};

#define DECLARE_LOGGER(__LEVEL__)                                    \
  template <typename LoggerT, typename... Args>                      \
  void RS_##__LEVEL__(LoggerT &&, const char * fmt, Args &&... args) \
  {                                                                  \
    if constexpr (sizeof...(args) == 0) {                            \
      ROS_##__LEVEL__("%s", fmt);                                    \
    } else {                                                         \
      ROS_##__LEVEL__(fmt, std::forward<Args>(args)...);             \
    }                                                                \
  }                                                                  \
                                                                     \
  template <typename LoggerT, typename T>                            \
  void RS_##__LEVEL__##_STREAM(LoggerT &&, T && msg)                 \
  {                                                                  \
    ROS_##__LEVEL__##_STREAM(std::forward<T>(msg));                  \
  }

#endif
}  // namespace robosense::rs_monitor

DECLARE_LOGGER(INFO)
DECLARE_LOGGER(WARN)
DECLARE_LOGGER(ERROR)
DECLARE_LOGGER(FATAL)
DECLARE_LOGGER(DEBUG)

#endif  // RS_MONITOR_ROS_ADAPTER_H
