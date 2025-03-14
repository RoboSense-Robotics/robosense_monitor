#ifndef RS_MONITOR_COMMON_H
#define RS_MONITOR_COMMON_H

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <string>
#include <string_view>
#include <vector>
#include <cstdint>

#include "rs_monitor/common/ros_adapter.h"

namespace robosense::rs_monitor
{

// static variables
constexpr float kFloatPrecision = 1e-3;

// static methods
/**
 * @brief Gets the full path to a package configuration file
 *
 * Combines the package share directory with the provided suffix to
 * construct a path to configuration files. Validates that the path
 * exists and is a regular file.
 *
 * @param nh ROS node handle (used for parameter access and logging)
 * @param config_file_suffix Relative path within the package
 * @return Full path to the config file, or empty string if not found
 */
std::string get_config_path(NodeHandle & nh, std::filesystem::path const & config_file_suffix);

static YAML::Node load_config(std::string const & config_path)
{
  try {
    return YAML::LoadFile(config_path);
  } catch (YAML::Exception const & e) {
    static_cast<void>(e);
    return YAML::Node(YAML::NodeType::Undefined);
  }
}

template <bool ThrowException = false, bool AbsolutePath = false>
YAML::Node load_config(NodeHandle & nh, std::filesystem::path const & config_file_suffix)
{
  std::string config_path;

  if constexpr (AbsolutePath) {
    config_path = std::filesystem::absolute(config_file_suffix).string();
  } else {
    config_path = get_config_path(nh, config_file_suffix);
  }

  if (!std::filesystem::exists(config_path)) {
    if constexpr (ThrowException) {
      throw std::runtime_error("Config file not found at [" + config_path + "]!");
    } else {
      RS_ERROR(nh, "Config file not found at [%s]!", config_path.c_str());
      return YAML::Node(YAML::NodeType::Undefined);
    }
  }

  auto config(load_config(config_path));

  if (!config) {
    std::string config_load_error("Failed to parse config file at [");
    config_load_error.append(config_path).append("]!");

    if constexpr (ThrowException) {
      throw std::runtime_error(config_load_error);
    } else {
      RS_ERROR(nh, config_load_error.c_str());
      return config;
    }
  }

  RS_INFO(
    nh, "Config file Loaded from %s:\n[CONFIG BEGIN]\n%s\n[CONFIG END]", config_path.c_str(),
    YAML::Dump(config).c_str());

  return config;
}

/**
 * @brief Parses a uint64_t value from a string
 *
 * Extracts the first sequence of digits from the input string and converts
 * it to a uint64_t value. Useful for parsing values from /proc filesystem.
 *
 * @param line String to parse
 * @return Extracted uint64_t value, or 0 if no digits found
 */
uint64_t parse_uint64_in_string(std::string const & line);

/**
 * @brief Splits a string into substrings based on a delimiter
 *
 * @tparam T Return container type (must support push_back)
 * @param str String to split
 * @param delimiter Character to use as delimiter
 * @return Container of substrings
 */
template <class String>
std::vector<std::string_view> split_string(String const & str, char const splitter = ' ');

}  // namespace robosense::rs_monitor

#endif  // RS_MONITOR_COMMON_H
