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

#include "rs_monitor/monitor/frequency_monitor.h"

#include "rs_monitor/common/common.h"
#include "diagnostic_updater/diagnostic_updater.hpp"

namespace robosense::rs_monitor
{

FrequencyMonitor::FrequencyMonitor(NodeHandle * nh) : MonitorBase("frequency_monitor", nh) {}

bool FrequencyMonitor::init(YAML::Node const & config)
{
  if (!MonitorBase::init(config)) {
    return false;
  }

  // 读取 topic 列表
  try {
    auto frequency_monitor_config = config[get_name()];
    if (!frequency_monitor_config || !frequency_monitor_config.IsMap()) {
      RS_ERROR(get_node(), "Missing required map field [%s] in config file!", get_name().c_str());
      return false;
    }

    auto topics = frequency_monitor_config["topics"];
    if (!topics || !topics.IsSequence()) {
      RS_ERROR(get_node(), "No topics configured or invalid format");
      return false;
    }

    auto & updater(get_updater());
    updater.setPeriod(std::round(get_exec_interval_ms() / 1e3));
    updater.add(
      "Topic Frequencies",
      std::bind(&FrequencyMonitor::update_frequency_status, this, std::placeholders::_1));

    // 为每个通道创建订阅者
    for (auto const & topic : topics) {
      std::string topic_name = topic["name"].as<std::string>();
      float min_freq = topic["min_freq"].as<float>();

      auto & topic_config = topics_[topic_name];

      topic_config.name = topic_name;
      topic_config.min_freq = min_freq;
    }
  } catch (std::exception const & e) {
    RS_ERROR(get_node(), "Failed to parse config fields: %s", e.what());
    return false;
  }

  prepare_generic_subscribers<FrequencyMonitor, FirstArgType::TopicName>(
    topics_, subscribers_, &FrequencyMonitor::message_callback);

  return true;
}

void FrequencyMonitor::message_callback(
  CALLBACK_PARAM_TYPE(SerializedMessage), std::string const & topic_name)
{
  auto it = topics_.find(topic_name);
  if (it != topics_.end()) {
    it->second.message_count.fetch_add(1, std::memory_order_relaxed);
  }
}

uint64_t FrequencyMonitor::now_ms() { return SystemTime() / 1e6; }

void FrequencyMonitor::calculate_frequencies()
{
  auto current_timestamp_ms = now_ms();

  for (auto & [topic, config] : topics_) {
    if (config.is_enabled) {
      float interval_sec = 0.0f;
      if (config.last_calculate_time == 0) {
        interval_sec = get_exec_interval_ms() * 1e-3;
      } else {
        interval_sec = (current_timestamp_ms - config.last_calculate_time) * 1e-3;
      }

      if (interval_sec > 0) {
        // 计算当前采样的帧率
        auto message_count = config.message_count.load(std::memory_order_acquire);
        config.message_count.fetch_sub(message_count, std::memory_order_acq_rel);

        float current_sample = message_count / interval_sec;

        // 更新历史数据
        config.freq_history[config.history_index] = current_sample;
        config.history_index = (config.history_index + 1) % TopicConfig::WINDOW_SIZE;
        config.valid_samples = std::min(config.valid_samples + 1, TopicConfig::WINDOW_SIZE);

        if (config.valid_samples < TopicConfig::WINDOW_SIZE) {
          continue;
        }

        // 计算平均帧率
        float sum = 0.0f;
        for (size_t i = 0; i < config.valid_samples; ++i) {
          sum += config.freq_history[i];
        }
        config.current_frequency = sum / config.valid_samples;

        if (config.min_freq - config.current_frequency > kFloatPrecision) {
          RS_ERROR(
            get_node(), "Low frequency on topic [%s]: %.10f Hz (min: %.10f Hz)", topic.c_str(),
            config.current_frequency, config.min_freq);
        }
      }

      // 重置消息计数和时间戳
      config.last_calculate_time = current_timestamp_ms;
    }
  }
}

void FrequencyMonitor::run_once(uint64_t)
{
#if __ROS1__
  auto ptr = &get_updater();
  if (ptr) {
    ptr->update();
  }
#endif
}

void FrequencyMonitor::update_frequency_status(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  calculate_frequencies();
  prepare_generic_subscribers<FrequencyMonitor, FirstArgType::TopicName>(
    topics_, subscribers_, &FrequencyMonitor::message_callback);

  bool all_ok = true;
  std::string message = "Topic frequencies OK";

  for (const auto & [topic, config] : topics_) {
    if (!config.is_enabled || !config.last_calculate_time) {
      continue;
    }
    if (config.min_freq - config.current_frequency > kFloatPrecision) {
      all_ok = false;
      message = "Low frequency detected";
    }

    stat.add(topic, std::to_string(config.current_frequency) + " Hz");
  }

  stat.summary(all_ok ? DiagnosticStatus::OK : DiagnosticStatus::WARN, message);
}

}  // namespace robosense::rs_monitor
