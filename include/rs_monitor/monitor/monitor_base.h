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

#include <atomic>
#include <memory>
#include <string>
#include <cstdint>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include <yaml-cpp/yaml.h>

#include <rclcpp/qos.hpp>
#include <rclcpp/node_interfaces/node_graph.hpp>

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

template <typename T, typename = void>
struct HasMemberSubscriptionFailure : std::false_type
{
};

template <typename T>
struct HasMemberSubscriptionFailure<
  T, std::void_t<decltype(std::declval<T>().has_subscription_failure)>> : std::true_type
{
};

template <typename T, typename = void>
struct HasMemberLastCalculateTime : std::false_type
{
};

template <typename T>
struct HasMemberLastCalculateTime<T, std::void_t<decltype(std::declval<T>().last_calculate_time)>>
: std::true_type
{
};

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
  using CallbackType = std::function<void(CALLBACK_PARAM_TYPE(SerializedMessage) const &)>;

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
  inline std::string const & get_owner_id() const { return owner_id_; }
  inline uint64_t get_exec_interval_ms() const { return exec_interval_ms_; }
  inline diagnostic_updater::Updater & get_updater() const { return *updater_; }

#if __ROS2__
  // Publish a quickdata request (std_msgs/String JSON) with a global cooldown gate.
  // Returns true if published; false if suppressed by cooldown or publisher unavailable.
  bool publish_quickdata_request(std::string const & issue_type, std::string const & issue_text);
#endif

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
   * @param topics Map of topic configurations
   * @param callback Member function to call when messages are received
   */
  template <typename This, typename TopicMap, typename F>
  void prepare_generic_subscribers(
    TopicMap & topics, F && callback, std::string const & owner_id = {})
  {
    auto const owner_key = owner_id.empty() ? this->get_owner_id() : owner_id;
    for (auto & [topic, config] : topics) {
      if (config.is_enabled) {
        continue;
      }

      if constexpr (HasMemberSubscriptionFailure<decltype(config)>::value) {
        if (config.has_subscription_failure) {
          continue;
        }
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

      // 生成 bind 后的回调函数
      auto bound_cb = [this, &config, cb = std::forward<F>(callback)](
                        CALLBACK_PARAM_TYPE(SerializedMessage) const & msg) mutable {
        using CB = std::decay_t<decltype(cb)>;
        if constexpr (std::is_member_function_pointer_v<CB>) {
          (reinterpret_cast<This *>(this)->*cb)(msg, config);
        } else {
          cb(msg, config);
        }
      };

      std::lock_guard lock(subscriber_hub_mtx_);
      auto it = subscriber_hub_.find(topic);
      if (it != subscriber_hub_.end()) {
        it->second.callbacks[owner_key] = bound_cb;
        config.is_enabled = true;
        // 添加 callback 后的 config 后处理
        if constexpr (HasMemberLastCalculateTime<decltype(config)>::value) {
          config.last_calculate_time = SteadyTime() / 1e6;  // unit: ms
        }
        continue;
      }

#if __ROS2__
      // 动态获取 QoS, 取兼容性最高的结果
      auto qos = [&]() {
        // 统一使用 KeepLast, depth 取发布端最大值和 depth 的较大者
        size_t depth = 100;
        auto reliability = rclcpp::ReliabilityPolicy::Reliable;  // 有 BE 则用 BE, 否则用 Reliable
        auto durability = rclcpp::DurabilityPolicy::Volatile;  // 有 TransientLocal 则提升

        auto graph = this->get_node().get_node_graph_interface();
        auto infos = graph->get_publishers_info_by_topic(topic, /*no_demangle=*/false);

        for (auto const & ep : infos) {
          auto const & pq = ep.qos_profile();
          depth = std::max(depth, static_cast<size_t>(pq.depth()));
          if (pq.reliability() == rclcpp::ReliabilityPolicy::BestEffort) {
            reliability = rclcpp::ReliabilityPolicy::BestEffort;
          }
        }

        rclcpp::QoS q{rclcpp::KeepLast(depth)};
        q.reliability(reliability).durability(durability);
        return q;
      }();

      GenericSubscriber subscriber;

#define ___CREATE_CALLBACK(__callback__)                                                        \
  try {                                                                                         \
    subscriber =                                                                                \
      this->get_node().create_generic_subscription(topic, topic_type, qos, __callback__);       \
  } catch (const std::exception & e) {                                                          \
    RS_ERROR(                                                                                   \
      get_node(), "Failed to create subscription for topic [%s]: %s", topic.c_str(), e.what()); \
    if constexpr (HasMemberSubscriptionFailure<decltype(config)>::value) {                      \
      config.has_subscription_failure = true;                                                   \
    }                                                                                           \
  }

#elif __ROS1__
#define ___CREATE_CALLBACK(__callback__) \
  *subscriber = this->get_node().subscribe<topic_tools::ShapeShifter>(topic, 1, __callback__)
      GenericSubscriber subscriber(new ros::Subscriber());
#endif

      bool success = false;

      subscriber_hub_[topic] = SubscriberEntry{};
      auto & callbacks = subscriber_hub_[topic].callbacks;
      callbacks.emplace(owner_key, std::move(bound_cb));

      // 生成 dispatcher, 批量调用已注册的回调函数
      auto dispatch = [this, topic](CALLBACK_PARAM_TYPE(SerializedMessage) const & msg) -> void {
        std::vector<CallbackType> snapshot;
        {
          std::lock_guard lock(subscriber_hub_mtx_);
          auto it = subscriber_hub_.find(topic);
          if (it == subscriber_hub_.end()) {
            return;
          }
          snapshot.reserve(it->second.callbacks.size());
          for (auto & [_, fn] : it->second.callbacks) {
            snapshot.push_back(fn);
          }
        }
        for (auto & fn : snapshot) {
          fn(msg);
        }
      };

      ___CREATE_CALLBACK(dispatch);
      success = subscriber != nullptr;
#undef ___CREATE_CALLBACK

      if (!success) {
        continue;
      }

      // 添加 callback 后的 config 后处理 [同上]
      config.is_enabled = true;
      if constexpr (HasMemberLastCalculateTime<decltype(config)>::value) {
        config.last_calculate_time = SteadyTime() / 1e6;  // unit: ms
      }
      subscriber_hub_[topic].subscriber = std::move(subscriber);
    }
  }

  bool remove_subscriber_callback(std::string const & topic, std::string const & owner_id)
  {
    if (owner_id.empty()) {
      return false;
    }
    std::lock_guard lock(subscriber_hub_mtx_);
    auto it = subscriber_hub_.find(topic);
    if (it == subscriber_hub_.end()) {
      return false;
    }
    auto & callbacks = it->second.callbacks;
    auto erased = callbacks.erase(owner_id) > 0;
    if (callbacks.empty()) {
      it->second.subscriber.reset();
      subscriber_hub_.erase(it);
    }
    return erased;
  }

protected:
  std::string hardware_id_{"none"};

private:
  // std::weak_ptr<rclcpp::Node> node_;
  NodeHandle * const node_handle_ptr_{nullptr};
  std::shared_ptr<diagnostic_updater::Updater> updater_;

  uint64_t exec_interval_ms_ = UINT64_MAX;
  uint64_t last_exec_timestamp_ms_ = 0;
  std::string monitor_name_{};
  std::string owner_id_{};
  std::string publish_topic_name_{};
  bool is_enabled_;

  struct SubscriberEntry
  {
    GenericSubscriber subscriber{nullptr};
    std::unordered_map<std::string, CallbackType> callbacks{};
  };

  inline static std::mutex subscriber_hub_mtx_{};
  inline static std::unordered_map<std::string, SubscriberEntry> subscriber_hub_{};
};

}  // namespace robosense::rs_monitor

#endif  // RS_MONITOR_MONITOR_BASE_H
