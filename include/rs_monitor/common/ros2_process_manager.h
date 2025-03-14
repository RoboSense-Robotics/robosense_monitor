#ifndef RS_MONITOR_COMMON_ROS2_PROCESS_MANAGER_H
#define RS_MONITOR_COMMON_ROS2_PROCESS_MANAGER_H

#include <yaml-cpp/yaml.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#ifdef _WIN32
#define pid_t DWORD
#include <Windows.h>
#endif

#include "rs_monitor/common/ros_adapter.h"

namespace robosense::rs_monitor
{

struct ROS2ProcessInfo
{
  std::string name;
  std::vector<std::string> node_names;
  pid_t pid;
  ros_time_t last_update{0};
  bool is_monitored{false};
};

/**
 * @class ROS2ProcessManager
 * @brief Singleton class for detecting and managing ROS2 processes
 *
 * Identifies ROS2 processes in the system and maintains information about them
 * such as PID, node names, and other relevant details.
 */
class ROS2ProcessManager
{
public:
  /**
   * @brief Gets the singleton instance of the ROS2ProcessManager
   * @return Reference to the singleton instance
   */
  static ROS2ProcessManager & instance();

  /**
   * @brief Initializes the process manager with configuration from YAML
   *
   * Configures process identification patterns and monitoring settings.
   *
   * @param config YAML configuration node
   * @param node ROS2 node for logging
   * @return true if initialization succeeded, false otherwise
   */
  bool init(YAML::Node const & config, NodeHandle & node);

  /**
   * @brief Gets the currently detected ROS2 processes
   * @return Map of process IDs to process information
   */
  std::unordered_map<int, ROS2ProcessInfo> get_processes();

private:
  inline static std::vector<std::string> const kDefaultROS2Identifiers = {"--ros-args", "_node:"};
  inline static std::vector<std::string> const kDefaultROS2Comm = {"ros2"};
  static constexpr size_t kDefaultMaxProcessNameLength = 40;
  static constexpr int64_t kMinimumRefreshIntervalMillSec = 1000;

  struct MonitoredProcess
  {
    std::string keyword;
    std::string name;
  };

private:
  ROS2ProcessManager() = default;
  ~ROS2ProcessManager() = default;

  // Default identifiers and settings

  /**
   * @brief Refreshes the cache of detected ROS2 processes
   *
   * Scans the system for processes matching ROS2 patterns and
   * updates the internal process list.
   */
  void refresh_process_cache();

  /**
   * @brief Determines if a process is a ROS2 process based on command line and executable
   * @param cmdline Process command line
   * @param comm Process executable name
   * @return true if the process is a ROS2 process, false otherwise
   */
  bool is_ros2_process(std::string const & cmdline, std::string const & comm) const;

  /**
   * @brief Checks if a process is still running
   * @param pid Process ID to check
   * @return true if the process is running, false otherwise
   */
  bool is_process_alive(int pid) const;

  /**
   * @brief Reads the executable name of a process
   * @param pid Process ID
   * @return Process executable name
   */
  std::string read_comm(pid_t pid) const;

  /**
   * @brief Reads the command line of a process
   * @param pid Process ID
   * @return Process command line
   */
  std::string read_cmd_line(pid_t pid) const;

  /**
   * @brief Derives a display name for a process
   * @param cmdline Process command line
   * @param comm Process executable name
   * @return Display name for the process
   */
  std::string get_process_name(std::string const & cmdline, std::string const & comm) const;

  /**
   * @brief Extracts ROS2 node names from a command line
   * @param cmdline Process command line
   * @return Vector of node names
   */
  std::vector<std::string> get_node_names(std::string const & cmdline) const;

  /**
   * @brief Checks if a process matches any monitored process pattern
   * @param cmdline Process command line
   * @param monitored_processes List of monitored process patterns
   * @return Pair of (is_monitored, process_name)
   */
  std::pair<bool, std::string> is_monitored_process(
    std::string const & cmdline, std::vector<MonitoredProcess> & monitored_processes) const;

  NodeHandle * nh_{nullptr};
  ros_time_t last_refresh_time_{0};
  std::atomic<bool> is_initialized{false};
  std::atomic_flag refresh_flag_{ATOMIC_FLAG_INIT};
  ROSTimer refresh_timer_;
  std::vector<std::string> process_identifiers_comm_;
  std::vector<std::string> process_identifiers_cmdline_;
  std::unordered_map<int, ROS2ProcessInfo> processes_;

  uint64_t refresh_interval_ms_{6000};
  size_t max_process_name_length_;
  std::vector<MonitoredProcess> monitored_processes_;
};

}  // namespace robosense::rs_monitor

#endif  // RS_MONITOR_COMMON_ROS2_PROCESS_MANAGER_H
