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

// monitor.hpp
#ifndef RS_MONITOR_MONITOR_BASE_H
#define RS_MONITOR_MONITOR_BASE_H

#include <memory>
#include <type_traits>

#include <yaml-cpp/yaml.h>

#include "rs_monitor/common/ros_adapter.h"
#include "rs_monitor/common/common.h"

namespace rclcpp
{
class Node;
class Logger;
template <typename MessageType, typename AllocatorT>
class Publisher;
}  // namespace rclcpp

namespace diagnostic_updater
{
class Updater;
}  // namespace diagnostic_updater

namespace robosense::rs_monitor
{

/**
 * @class MonitorBase
 * @brief Base class for all monitoring modules
 *
 * Provides common functionality for monitoring modules including
 * initialization, periodic execution, and diagnostic publishing.
 */
class MonitorBase
{
public:
  /**
   * @brief Constructor for MonitorBase
   *
   * @param name Name of the monitor
   * @param nh Pointer to the node handle
   */
  explicit MonitorBase(std::string const & monitor_name, NodeHandle * nh);

  /**
   * @brief Virtual destructor
   */
  virtual ~MonitorBase() = default;

  /**
   * @brief Updates the monitor with the current timestamp
   *
   * This method is called periodically to update the monitor's state
   * and perform any necessary calculations.
   *
   * @param current_timestamp_ms Current timestamp in milliseconds
   */
  void update(uint64_t current_timestamp_ms);

  /**
   * @brief Initializes the monitor with configuration
   *
   * Base implementation sets up common parameters like execution interval
   * and message publisher.
   *
   * @param config YAML configuration node
   * @return true if initialization succeeded, false otherwise
   */
  virtual bool init(YAML::Node const & config);

protected:
  virtual void run_once(uint64_t current_timestamp_ms) = 0;

  inline NodeHandle & get_node() const { return *node_handle_ptr_; }
  inline std::string const & get_name() const { return monitor_name_; }
  inline uint64_t get_exec_interval_ms() const { return exec_interval_ms_; }
  inline diagnostic_updater::Updater & get_updater() const { return *updater_; }

  enum class FirstArgType : uint8_t {
    None,
    TopicName,
    TopicConfig,
  };

  /**
   * @brief Template method to prepare generic subscribers
   *
   * Sets up generic message subscribers for a set of topics with appropriate callbacks.
   *
   * @tparam This Type of the derived class
   * @tparam FAT First argument type for the callback
   * @param topics Map of topic configurations
   * @param subscribers Map to store created subscribers
   * @param callback Member function to call when messages are received
   */
  template <typename This, FirstArgType FAT, typename TopicMap, typename SubscriberMap, typename F>
  void prepare_generic_subscribers(TopicMap & topics, SubscriberMap & subscribers, F && callback)
  {
    if (subscribers.size() == topics.size()) {
      return;
    }

    for (auto & [topic, config] : topics) {
      auto it = subscribers.find(topic);
      if (it != subscribers.end() && it->second) {
        continue;
      }

      bool topic_has_publisher = false;
      std::string topic_type{};

#if __ROS2__
      auto publications(this->get_node().get_publishers_info_by_topic(topic));
      if (!publications.empty()) {
        topic_has_publisher = true;
        topic_type = publications[0].topic_type();
      }
#elif __ROS1__
      ros::master::V_TopicInfo topic_info_list;
      ros::master::getTopics(topic_info_list);
      for (auto const & topic_info : topic_info_list) {
        if (topic_info.name == topic) {
          topic_has_publisher = true;
          topic_type = topic_info.datatype;
          break;
        }
      }
#endif

      if (!topic_has_publisher) {
        continue;
      }

      // 创建 subscriber
#if __ROS2__
      auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
      GenericSubscriber subscriber;
#define ___CREATE_CALLBACK(__callback__) \
  subscriber = this->get_node().create_generic_subscription(topic, topic_type, qos, __callback__)

#elif __ROS1__
#define ___CREATE_CALLBACK(__callback__) \
  *subscriber = this->get_node().subscribe<topic_tools::ShapeShifter>(topic, 1, __callback__)
      GenericSubscriber subscriber(new ros::Subscriber());

#endif

      // 根据回调类型和 FirstArgType 创建适当的回调
      bool success = false;

      if constexpr (std::is_member_function_pointer<F>::value) {
        if constexpr (std::is_base_of_v<MonitorBase, This>) {
          if constexpr (FAT == FirstArgType::TopicName) {
            auto cb = [callback, topic, this](CALLBACK_PARAM_TYPE(SerializedMessage) const & msg) {
              (reinterpret_cast<This *>(this)->*callback)(msg, topic);
            };
            ___CREATE_CALLBACK(cb);
            success = subscriber != nullptr;
          } else if constexpr (FAT == FirstArgType::TopicConfig) {
            auto cb = [callback, config, this](CALLBACK_PARAM_TYPE(SerializedMessage) const & msg) {
              (reinterpret_cast<This *>(this)->*callback)(msg, config);
            };
            ___CREATE_CALLBACK(cb);
            success = subscriber != nullptr;
          }
        }
      }
#undef ___CREATE_CALLBACK
      if (!success) {
        RS_ERROR(this->get_node(), "Failed to create subscription for topic [%s]", topic.c_str());
        continue;
      }

      config.is_enabled = true;
      subscribers[topic] = std::move(subscriber);
    }
  }

private:
  // std::weak_ptr<rclcpp::Node> node_;
  NodeHandle * const node_handle_ptr_{nullptr};
  std::shared_ptr<diagnostic_updater::Updater> updater_;

  uint64_t exec_interval_ms_ = UINT64_MAX;
  uint64_t last_exec_timestamp_ms_ = 0;
  std::string monitor_name_{};
  std::string publish_topic_name_{};
  bool is_enabled_;
};

}  // namespace robosense::rs_monitor

#endif  // RS_MONITOR_MONITOR_BASE_H
