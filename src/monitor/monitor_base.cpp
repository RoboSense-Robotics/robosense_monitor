#include "rs_monitor/monitor/monitor_base.h"

#include "diagnostic_updater/diagnostic_updater.hpp"

namespace robosense::rs_monitor
{

MonitorBase::MonitorBase(std::string const & monitor_name, NodeHandle * nh)
: node_handle_ptr_(nh), monitor_name_(monitor_name)
{
}

void MonitorBase::update(uint64_t current_time_ms)
{
  if (
    last_exec_timestamp_ms_ != 0 &&
    current_time_ms - last_exec_timestamp_ms_ < exec_interval_ms_ * 0.95) {
    return;
  }
  last_exec_timestamp_ms_ = current_time_ms;

  this->run_once(current_time_ms);
}

bool MonitorBase::init(YAML::Node const & config)
{
  last_exec_timestamp_ms_ = 0;

  if (!node_handle_ptr_) {
    return false;
  }

  // load config
  if (!config.IsMap()) {
    RS_ERROR(node_handle_ptr_, "Root object in config file is not a map!");
    return false;
  }

  auto monitor_config(config[monitor_name_]);
  if (!monitor_config.IsDefined()) {
    RS_ERROR(node_handle_ptr_, "Missing key [%s] in config file!", monitor_name_.c_str());
    return false;
  }

  if (!monitor_config.IsMap()) {
    RS_ERROR(node_handle_ptr_, "Config of [%s] is not a map!", monitor_name_.c_str());
    return false;
  }

  try {
    is_enabled_ = monitor_config["enable"].as<bool>();
    exec_interval_ms_ = monitor_config["exec_interval_ms"].as<uint64_t>();
  } catch (YAML::Exception const & e) {
    RS_ERROR(node_handle_ptr_, "Failed to parse config fields: %s", e.what());
    return false;
  }

  if (!is_enabled_) {
    RS_ERROR(node_handle_ptr_, "[%s] has been disabled", monitor_name_.c_str());
    return false;
  }

  auto publish_topic_name = monitor_config["publish_topic_name"];
  if (publish_topic_name) {
    if (publish_topic_name.IsScalar()) {
      // 只在 publish_topic_name 可转换为字符串时创建 diagnostic updater
      publish_topic_name_ = publish_topic_name.as<std::string>();
      if (publish_topic_name_.empty()) {
        RS_ERROR(node_handle_ptr_, "Required field [publish_topic_name] can not be empty!");
        return false;
      }

      updater_ =
        std::make_shared<diagnostic_updater::Updater>(node_handle_ptr_, publish_topic_name_);
      if (!updater_) {
        RS_ERROR(node_handle_ptr_, "Failed to create diagnostic updater!");
        return false;
      }
      updater_->setHardwareID("none");
    }
  } else {
    RS_ERROR(node_handle_ptr_, "Missing required key [publish_topic_name]!");
    return false;
  }

  return true;
}

}  // namespace robosense::rs_monitor
