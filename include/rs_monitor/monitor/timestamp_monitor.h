#ifndef RS_MONITOR_TIMESTAMP_MONITOR_H
#define RS_MONITOR_TIMESTAMP_MONITOR_H

#include <memory>
#include <unordered_map>

#include "rs_monitor/monitor/monitor_base.h"

namespace robosense::rs_monitor
{

/**
 * @class TimestampMonitor
 * @brief Monitors timestamp differences between received messages and system time
 *
 * Tracks the time difference between message timestamps and the time of
 * message reception, reporting issues when the difference exceeds thresholds.
 */
class TimestampMonitor : public MonitorBase
{
public:
  /**
   * @brief Constructor for TimestampMonitor
   * @param nh Pointer to the generic node handle
   */
  explicit TimestampMonitor(NodeHandle * nh);

  /**
   * @brief Destructor
   */
  ~TimestampMonitor() override = default;

  /**
   * @brief Initializes the timestamp monitor
   *
   * Configures topics to monitor and their maximum allowed time differences
   * from the provided YAML configuration.
   *
   * @param config YAML configuration node
   * @return true if initialization succeeded, false otherwise
   */
  bool init(YAML::Node const & config) override;

protected:
  /**
   * @struct TopicConfig
   * @brief Configuration and runtime data for timestamp monitoring
   *
   * Stores both configuration parameters and runtime metrics for a monitored topic.
   */
  struct TopicConfig
  {
    uint32_t max_difference_ms{0};
    std::string name{};
    bool is_enabled{false};
  };

  /**
   * @brief Periodic execution function
   * @param current_timestamp_ms Current timestamp in milliseconds
   */
  void run_once(uint64_t current_timestamp_ms) override;

  /**
   * @brief Callback for received messages on monitored topics
   *
   * Checks the timestamp difference between the message and the system time
   * and logs warnings if it exceeds the configured threshold.
   *
   * @param message Serialized message
   * @param config Topic configuration
   */
  void callback(CALLBACK_PARAM_TYPE(SerializedMessage) const & message, TopicConfig const & config);

private:
  std::unordered_map<std::string, TopicConfig> topics_;
  GenericSubscriberMap subscribers_;
};

}  // namespace robosense::rs_monitor

#endif  // RS_MONITOR_TIMESTAMP_MONITOR_H
