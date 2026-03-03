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

namespace robosense::rs_monitor
{

TimestampMonitor::TimestampMonitor(NodeHandle * nh) : MonitorBase("timestamp_monitor", nh) {}

void TimestampMonitor::run_once(uint64_t)
{
  prepare_generic_subscribers<TimestampMonitor>(topics_, &TimestampMonitor::callback);
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

    auto & updater(get_updater());
    updater.setPeriod(get_exec_interval_ms() / 1e3);
    updater.add(
      "Abnormal Latency Status",
      std::bind(&TimestampMonitor::update_timestamp_status, this, std::placeholders::_1));

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
      topic_config.message_count = 0;
      topic_config.abnormal_latency_count = 0;
    }

  } catch (std::exception const & e) {
    RS_ERROR(get_node(), "Failed to parse config fields: %s", e.what());
    return false;
  }

  prepare_generic_subscribers<TimestampMonitor>(topics_, &TimestampMonitor::callback);

  return true;
}

void TimestampMonitor::callback(
  CALLBACK_PARAM_TYPE(SerializedMessage) const & message, TimestampMonitor::TopicConfig & config)
{
  if (!config.is_deserializable) {
    return;
  }

  uint64_t now_ns = ROSTime();

  // extract timestamp from message header
#if __ROS2__
  std_msgs::msg::Header header;
  try {
    rclcpp::Serialization<std_msgs::msg::Header> serialization;
    serialization.deserialize_message(message.get(), &header);
  } catch (rclcpp::exceptions::RCLError const & re) {
    RS_ERROR(
      get_node(), "Failed to deserialize header in topic [%s]: %s", config.name.c_str(), re.what());
    config.is_deserializable = false;
    return;
  }

  uint64_t const timestamp_ns = header.stamp.sec * 1000000000ull + header.stamp.nanosec;
#elif __ROS1__
  std_msgs::Header header;
  std::vector<uint8_t> buf(message->size());
  ros::serialization::OStream stream(buf.data(), message->size());
  message->write(stream);

  header.seq = ((uint32_t *)buf.data())[0];
  header.stamp.sec = ((uint32_t *)buf.data())[1];
  header.stamp.nsec = ((uint32_t *)buf.data())[2];

  if (cache_it == has_std_header_mapping.end()) {
    if (abs((header.stamp.sec - (ROSTime() / 1e9))) >= 10.0) {
      RS_WARN(
        get_node(),
        "Message timestamp is more than 10 seconds later than system timestamp, topic: [%s]",
        config.name.c_str());
      RS_WARN(
        get_node(), "Message timestamp: %u, system timestamp: %.10f", header.stamp.sec,
        ROSTime() / 1e9);
      config.is_deserializable = false;
    }
  }

  uint64_t const timestamp_ns = header.stamp.sec * 1000000000ull + header.stamp.nsec;
#endif

  // diff with current system timestamp
  if (timestamp_ns > now_ns) {
    RS_ERROR(
      get_node(), "Message timestamp is later than system timestamp, topic: [%s]",
      config.name.c_str());
  } else {
    int64_t const diff_ns = static_cast<int64_t>(now_ns) - static_cast<int64_t>(timestamp_ns);
    double const difference_ms = static_cast<double>(diff_ns) / 1e6;
    if (difference_ms > static_cast<double>(config.max_difference_ms)) {
      config.abnormal_latency_count.fetch_add(1, std::memory_order_relaxed);
      RS_ERROR(
        get_node(),
        "The difference between system timestamp and message timestamp exceeds the threshold "
        "[%ums], difference: [%.3fms], topic: [%s]",
        config.max_difference_ms, difference_ms, config.name.c_str());
    }
  }

  config.message_count.fetch_add(1, std::memory_order_relaxed);
}

void TimestampMonitor::update_timestamp_status(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  bool all_ok = true;
  std::string message{};

  for (auto & [topic, config] : topics_) {
    if (!config.is_enabled) {
      continue;
    }

    uint64_t message_count = config.message_count.load(std::memory_order_acquire);
    uint64_t abnormal_latency_count = config.abnormal_latency_count.load(std::memory_order_acquire);

    char buf[64]{0};
    std::snprintf(buf, sizeof(buf), "%lu / %lu Messages", abnormal_latency_count, message_count);
    stat.add(topic, buf);

    if (abnormal_latency_count > 0 && all_ok) {
      all_ok = false;
      message = "Abnormal latency detected";
    }
  }

  stat.summary(all_ok ? DiagnosticStatus::OK : DiagnosticStatus::WARN, message);
}

}  // namespace robosense::rs_monitor
