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

#include "rs_monitor/common/common.h"

#if __ROS2__
#include <ament_index_cpp/get_package_share_directory.hpp>
#endif

namespace robosense::rs_monitor
{

uint64_t parse_uint64_in_string(std::string const & line)
{
  size_t end = line.find_last_of("0123456789");
  size_t begin = line.find_first_of("0123456789");
  if (end == std::string::npos || begin == std::string::npos) {
    return 0;
  }

  return std::stoul(line.substr(begin, end - begin + 1));
}

std::string get_config_path(NodeHandle & nh, std::filesystem::path const & config_file_suffix)
{
  namespace fs = std::filesystem;
  static std::string const kParameterName("package_name");

  std::string config_path{};

#if __ROS2__
  if (!nh.has_parameter(kParameterName)) {
    nh.declare_parameter<std::string>(kParameterName, "");
  }

  auto package_name(nh.get_parameter(kParameterName).as_string());
#elif __ROS1__
  std::string package_name("rs_monitor");
  nh.getParamCached(kParameterName, package_name);
#endif

  if (package_name.empty()) {
    return config_path;
  }

#if __ROS2__
  fs::path package_share_directory(ament_index_cpp::get_package_share_directory(nh.get_name()));
#elif __ROS1__
  fs::path package_share_directory(ros::package::getPath(package_name));
#endif
  if (!fs::is_directory(package_share_directory)) {
    RS_ERROR(
      nh,
      "Failed to open package share diectory [%s], path does not "
      "exist or is not a directory!",
      package_share_directory.c_str());
    return config_path;
  }

  fs::path _config_path(package_share_directory / config_file_suffix);
  if (!fs::is_regular_file(_config_path)) {
    RS_ERROR(
      nh,
      "Config file error at [%s]: file does not exis or is not a "
      "reugular file!",
      _config_path.c_str());
    return config_path;
  }

  config_path = _config_path.string();
  return config_path;
}

template <>
std::vector<std::string_view> split_string(std::string_view const & sv, char const splitter)
{
  std::vector<std::string_view> result{};

  bool collecting = false;
  size_t begin = 0;

  for (size_t i = 0, size = sv.size(); i < size; ++i) {
    if (sv[i] == splitter) {
      if (collecting) {
        collecting = false;
        result.emplace_back(std::string_view(sv.data() + begin, i - begin));
      }
    } else {
      if (!collecting) {
        begin = i;
        collecting = true;
      }
    }
  }

  if (collecting) {
    result.emplace_back(std::string_view(sv.data() + begin, sv.size() - begin));
  }

  return result;
}

template <>
std::vector<std::string_view> split_string(std::string const & str, char const splitter)
{
  return split_string(std::string_view(str), splitter);
}

}  // namespace robosense::rs_monitor
