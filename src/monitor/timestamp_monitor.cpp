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

#include "rs_monitor/monitor/timestamp_monitor.h"

#if __ROS2__
#include <rclcpp/type_adapter.hpp>
#include <rclcpp/serialization.hpp>
#include <std_msgs/msg/header.hpp>
#elif __ROS1__
#include <std_msgs/Header.h>
#endif

#include "rs_monitor/common/common.h"
#include "diagnostic_updater/diagnostic_updater.hpp"

namespace robosense::rs_monitor
{

TimestampMonitor::TimestampMonitor(NodeHandle * nh) : MonitorBase("timestamp_monitor", nh) {}

void TimestampMonitor::run_once(uint64_t)
{
  prepare_generic_subscribers<TimestampMonitor, FirstArgType::TopicConfig>(
    topics_, subscribers_, &TimestampMonitor::callback);
#if __ROS1__
  auto ptr = &get_updater();
  if (ptr) {
    ptr->update();
  }
#endif
}

bool TimestampMonitor::init(YAML::Node const & config)
{
  if (!MonitorBase::init(config)) {
    return false;
  }

  try {
    auto & timestamp_monitor_config = config[get_name()];
    if (!timestamp_monitor_config || !timestamp_monitor_config.IsMap()) {
      RS_ERROR(get_node(), "Missing required map field [%s] in config file!", get_name().c_str());
      return false;
    }

    auto topics = timestamp_monitor_config["topics"];
    if (!topics || !topics.IsSequence()) {
      RS_ERROR(get_node(), "No topics configured or invalid format");
      return false;
    }

    for (auto const & topic : topics) {
      std::string topic_name = topic["name"].as<std::string>();
      uint32_t max_difference_ms = topic["max_difference_ms"].as<uint32_t>();

      auto & topic_config = topics_[topic_name];

      topic_config.name = std::move(topic_name);
      topic_config.max_difference_ms = max_difference_ms;
    }

  } catch (std::exception const & e) {
    RS_ERROR(get_node(), "Failed to parse config fields: %s", e.what());
    return false;
  }

  prepare_generic_subscribers<TimestampMonitor, FirstArgType::TopicConfig>(
    topics_, subscribers_, &TimestampMonitor::callback);

  return true;
}

void TimestampMonitor::callback(
  CALLBACK_PARAM_TYPE(SerializedMessage) const & message,
  TimestampMonitor::TopicConfig const & config)
{
  static std::unordered_map<std::string, bool> has_std_header_mapping{};

  uint64_t now_ns = ROSTime();

  auto cache_it = has_std_header_mapping.find(config.name);
  if (cache_it != has_std_header_mapping.end() && !cache_it->second) {
    return;
  }

#if __ROS2__
  std_msgs::msg::Header header;
  try {
    rclcpp::Serialization<std_msgs::msg::Header> serialization;
    serialization.deserialize_message(message.get(), &header);
    has_std_header_mapping[config.name] = true;
  } catch (rclcpp::exceptions::RCLError const & re) {
    RS_ERROR(
      get_node(), "Failed to deserialize header in topic [%s]: %s", config.name.c_str(), re.what());
    has_std_header_mapping[config.name] = false;
    return;
  }

  uint64_t timestamp_ns = header.stamp.sec * 1e9 + header.stamp.nanosec;
#elif __ROS1__
  std_msgs::Header header;
  std::vector<uint8_t> buf(message->size());
  ros::serialization::OStream stream(buf.data(), message->size());
  message->write(stream);

  header.seq = ((uint32_t *)buf.data())[0];
  header.stamp.sec = ((uint32_t *)buf.data())[1];
  header.stamp.nsec = ((uint32_t *)buf.data())[2];

  if (cache_it == has_std_header_mapping.end()) {
    if (abs((header.stamp.sec - (ROSTime() / 1e9))) < 5.0) {
      has_std_header_mapping[config.name] = true;
    } else {
      has_std_header_mapping[config.name] = false;
    }
  }

  uint64_t timestamp_ns = header.stamp.sec * 1e9 + header.stamp.nsec;
#endif

  if (timestamp_ns > now_ns) {
    RS_ERROR(
      get_node(), "Message timestamp is later than system timestamp, topic: [%s]",
      config.name.c_str());
  } else {
    uint32_t difference_ms = (now_ns - timestamp_ns) * kFloatPrecision;
    if (difference_ms > config.max_difference_ms) {
      RS_ERROR(
        get_node(),
        "The difference between system timestamp and message timestamp exceeds the threshold "
        "[%ums], difference: [%ums], topic: [%s]",
        config.max_difference_ms, difference_ms, config.name.c_str());
    }
  }
}

}  // namespace robosense::rs_monitor
