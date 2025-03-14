#ifndef RS_MONITOR_MONITOR_FREQUENCY_MONITOR_H
#define RS_MONITOR_MONITOR_FREQUENCY_MONITOR_H

#include <array>
#include <atomic>
#include <string>
#include <unordered_map>

#include "rs_monitor/monitor/monitor_base.h"
#include "diagnostic_updater/diagnostic_updater.hpp"

namespace robosense::rs_monitor
{

/**
 * @struct TopicConfig
 * @brief Configuration and runtime data for topic frequency monitoring
 *
 * Stores both configuration parameters and runtime metrics for a monitored topic.
 */
struct TopicConfig
{
  static constexpr size_t WINDOW_SIZE = 3;        ///< Size of the frequency history window
  std::array<float, WINDOW_SIZE> freq_history{};  ///< Frequency history buffer
  size_t history_index{0};                        ///< Current index in the history buffer
  size_t valid_samples{0};                        ///< Number of valid samples collected

  // Arranged for cache line optimization
  std::atomic<uint64_t> message_count{0};  ///< Atomic counter for received messages
  uint64_t last_calculate_time{0};         ///< Timestamp of last frequency calculation
  std::string name{};                      ///< Topic name
  float min_freq{0.0};                     ///< Minimum allowed frequency threshold
  float current_frequency{nanf("0")};      ///< Current calculated frequency
  bool is_enabled{false};                  ///< Whether monitoring is enabled
};
// static_assert(TopicConfig::WINDOW_SIZE > 1);

/**
 * @class FrequencyMonitor
 * @brief Monitors the frequency of messages on ROS2 topics
 *
 * Tracks message rates on configured topics and reports when frequencies
 * fall below specified thresholds.
 */
class FrequencyMonitor : public MonitorBase
{
public:
  /**
   * @brief Constructor for FrequencyMonitor
   * @param nh Pointer to the ROS node handle
   */
  explicit FrequencyMonitor(NodeHandle * nh);

  /**
   * @brief Destructor
   */
  ~FrequencyMonitor() override = default;

  /**
   * @brief Initializes the frequency monitor
   *
   * Configures topics to monitor and their minimum frequency thresholds
   * from the provided YAML configuration.
   *
   * @param config YAML configuration node
   * @return true if initialization succeeded, false otherwise
   */
  bool init(YAML::Node const & config) override;

protected:
  /**
   * @brief Periodic execution function
   * @param current_timestamp_ms Current timestamp in milliseconds
   */
  void run_once(uint64_t current_timestamp_ms) override;

private:
  /**
   * @brief Updates diagnostic information with topic frequency status
   * @param stat Diagnostic status wrapper to update
   */
  void update_frequency_status(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /**
   * @brief Callback for received messages on monitored topics
   *
   * Increments the message counter for the topic when a message is received.
   *
   * @param message Serialized message (unused)
   * @param topic_name Name of the topic
   */
  void message_callback(CALLBACK_PARAM_TYPE(SerializedMessage), std::string const & topic_name);

  /**
   * @brief Gets the current time in milliseconds
   * @return Current time in milliseconds
   */
  uint64_t now_ms();

  /**
   * @brief Calculates current frequencies for all monitored topics
   *
   * For each topic, calculates the message frequency based on received
   * message count and elapsed time, using a moving average over multiple samples.
   * Warns if frequency falls below threshold.
   */
  void calculate_frequencies();

private:
  std::unordered_map<std::string, TopicConfig> topics_;
  GenericSubscriberMap subscribers_;
};

}  // namespace robosense::rs_monitor

#endif  // RS_MONITOR_MONITOR_FREQUENCY_MONITOR_H
