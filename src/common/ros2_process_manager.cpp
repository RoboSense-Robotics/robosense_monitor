#include "rs_monitor/common/ros2_process_manager.h"

#include <algorithm>
#include <filesystem>
#include <functional>
#include <string>
#include <string_view>
#include <thread>

#ifdef _WIN32
#include <psapi.h>
#include <tlhelp32.h>
#include <windows.h>
#include <winternl.h>
#elif __linux__
#include <unistd.h>
#include <signal.h>
#include <fstream>
#endif

#include "rs_monitor/common/common.h"

#ifdef _WIN32
char const kPathSplitter = '\\';
#else
char const kPathSplitter = '/';
#endif

namespace fs = std::filesystem;

namespace robosense::rs_monitor
{

ROS2ProcessManager & ROS2ProcessManager::instance()
{
  static ROS2ProcessManager instance;
  return instance;
}

bool ROS2ProcessManager::init(YAML::Node const & config, NodeHandle & nh)
{
  char const * kMissingRequiredConfigFieldErr =
    "Missing field [ros2_process_manager.%s] in config file!";

  if (is_initialized.load(std::memory_order_acquire)) {
    return true;
  }
  // call once... init failed? let it go
  is_initialized.store(true, std::memory_order_release);

  nh_ = &nh;

  try {
    auto process_manager_config = config["ros2_process_manager"];
    if (process_manager_config) {
      if (process_manager_config["refresh_interval_ms"]) {
        refresh_interval_ms_ = process_manager_config["refresh_interval_ms"].as<uint64_t>();
      } else {
        RS_WARN(nh_, kMissingRequiredConfigFieldErr, "refresh_interval_ms");
      }

      // cmdline features
      if (process_manager_config["ros2_process_identifiers"]) {
        process_identifiers_cmdline_ =
          process_manager_config["ros2_process_identifiers"].as<std::vector<std::string>>();
      } else {
        process_identifiers_cmdline_ = kDefaultROS2Identifiers;
        RS_WARN(nh_, kMissingRequiredConfigFieldErr, "ros2_process_identifiers");
      }

      // comm features
      if (process_manager_config["ros2_process_comm"]) {
        process_identifiers_comm_ =
          process_manager_config["ros2_process_comm"].as<std::vector<std::string>>();
      } else {
        process_identifiers_comm_ = kDefaultROS2Comm;
        RS_WARN(nh_, kMissingRequiredConfigFieldErr, "ros2_process_comm");
      }

      if (process_manager_config["monitored_processes"]) {
        for (const auto & process : process_manager_config["monitored_processes"]) {
          MonitoredProcess mp;
          mp.keyword = process["keyword"].as<std::string>();
          mp.name = process["name"].as<std::string>();
          monitored_processes_.push_back(mp);
        }
      } else {
        RS_WARN(nh_, kMissingRequiredConfigFieldErr, "monitored_processes");
      }

      // max length of process name
      if (process_manager_config["max_process_name_length"]) {
        max_process_name_length_ = process_manager_config["max_process_name_length"].as<size_t>();
      } else {
        max_process_name_length_ = kDefaultMaxProcessNameLength;
      }
    }

    if (refresh_interval_ms_ < 100 || refresh_interval_ms_ > INT64_MAX) {
      RS_WARN(nh_, "Field [ros2_process_manager.refresh_interval_ms] is out of range!");
      return false;
    }

#if __ROS2__
    refresh_timer_ = nh.create_wall_timer(
      std::chrono::milliseconds(refresh_interval_ms_),
      std::bind(&ROS2ProcessManager::refresh_process_cache, this));
#elif __ROS1__
    refresh_timer_ = nh.createWallTimer(
      ros::WallDuration(refresh_interval_ms_ / 1e3),
      [this](ros::WallTimerEvent const &) { this->refresh_process_cache(); });
#endif
  } catch (YAML::Exception const & e) {
    RS_ERROR(nh_, "Failed to parse config: %s", e.what());
    return false;
  } catch (std::exception const & e) {
    RS_ERROR(nh_, "Failed to initialize ROS2ProcessManager: %s", e.what());
    return false;
  }

  refresh_process_cache();

  return true;
}

std::unordered_map<int, ROS2ProcessInfo> ROS2ProcessManager::get_processes()
{
  while (refresh_flag_.test_and_set(std::memory_order_acquire)) {
    std::this_thread::yield();
  }

  std::unordered_map<int, ROS2ProcessInfo> temp{};
  std::string died_processes{};

  for (auto it = processes_.begin(); it != processes_.end();) {
    if (!is_process_alive(it->first)) {
      if (died_processes.empty()) {
        died_processes.append(
          "Refreshing process cache is triggered due to the death of the following processes:");
      }
      died_processes.append(" " + std::to_string(it->first));
      it = processes_.erase(it);
    } else {
      ++it;
    }
  }

  temp = processes_;

  refresh_flag_.clear(std::memory_order_release);

  if (!died_processes.empty()) {
    RS_WARN(nh_, died_processes.c_str());
    if (refresh_timer_) {
      if (
        SystemTime() > last_refresh_time_ &&
        refresh_interval_ms_ * 1e6 + last_refresh_time_ > kMinimumRefreshIntervalMillSec * 1e6) {
#if __ROS2__
        refresh_timer_->call();
#elif __ROS1__
        refresh_process_cache();
#endif
      }
    }
  }

  return temp;
}

#ifdef _WIN32

std::string ROS2ProcessManager::read_cmd_line(pid_t pid) const
{
  HANDLE hProcess = OpenProcess(PROCESS_QUERY_INFORMATION | PROCESS_VM_READ, FALSE, pid);
  if (hProcess == NULL) {
    return "";
  }

  // 使用动态获取NtQueryInformationProcess地址的方式
  typedef NTSTATUS(NTAPI * NtQueryInformationProcessPtr)(
    HANDLE, PROCESSINFOCLASS, PVOID, ULONG, PULONG);
  static NtQueryInformationProcessPtr NtQueryInformationProcess =
    (NtQueryInformationProcessPtr)GetProcAddress(
      GetModuleHandleW(L"ntdll.dll"), "NtQueryInformationProcess");

  if (!NtQueryInformationProcess) {
    CloseHandle(hProcess);
    return "";
  }

  PROCESS_BASIC_INFORMATION pbi;
  NTSTATUS status =
    NtQueryInformationProcess(hProcess, ProcessBasicInformation, &pbi, sizeof(pbi), NULL);
  if (status != 0) {
    CloseHandle(hProcess);
    return "";
  }

  PEB peb;
  if (!ReadProcessMemory(hProcess, pbi.PebBaseAddress, &peb, sizeof(peb), NULL)) {
    CloseHandle(hProcess);
    return "";
  }

  RTL_USER_PROCESS_PARAMETERS params;
  if (!ReadProcessMemory(hProcess, peb.ProcessParameters, &params, sizeof(params), NULL)) {
    CloseHandle(hProcess);
    return "";
  }

  std::wstring wcmd;
  wcmd.resize(params.CommandLine.Length / sizeof(WCHAR) + 1);
  if (!ReadProcessMemory(
        hProcess, params.CommandLine.Buffer, &wcmd[0], params.CommandLine.Length, NULL)) {
    CloseHandle(hProcess);
    return "";
  }

  // 转换宽字符到多字节
  int size_needed =
    WideCharToMultiByte(CP_UTF8, 0, wcmd.c_str(), (int)wcmd.size(), NULL, 0, NULL, NULL);
  std::string cmdline(size_needed, 0);
  WideCharToMultiByte(
    CP_UTF8, 0, wcmd.c_str(), (int)wcmd.size(), &cmdline[0], size_needed, NULL, NULL);

  CloseHandle(hProcess);
  return cmdline;
}

std::string ROS2ProcessManager::read_comm(pid_t pid) const
{
  HANDLE hProcess = OpenProcess(PROCESS_QUERY_INFORMATION | PROCESS_VM_READ, FALSE, pid);
  if (hProcess == NULL) {
    return "";
  }

  WCHAR exePath[MAX_PATH];
  if (GetProcessImageFileNameW(hProcess, exePath, MAX_PATH) == 0) {
    CloseHandle(hProcess);
    return "";
  }

  CloseHandle(hProcess);

  // 提取文件名
  std::wstring wpath(exePath);
  size_t pos = wpath.find_last_of(L"\\");
  if (pos != std::wstring::npos) {
    wpath = wpath.substr(pos + 1);
  }

  // 转换宽字符到多字节
  int size_needed =
    WideCharToMultiByte(CP_UTF8, 0, wpath.c_str(), (int)wpath.size(), NULL, 0, NULL, NULL);
  std::string comm(size_needed, 0);
  WideCharToMultiByte(
    CP_UTF8, 0, wpath.c_str(), (int)wpath.size(), &comm[0], size_needed, NULL, NULL);

  return comm;
}

bool ROS2ProcessManager::is_process_alive(int pid) const
{
  HANDLE hProcess = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, FALSE, pid);
  if (hProcess == NULL) {
    return false;
  }

  DWORD exitCode;
  GetExitCodeProcess(hProcess, &exitCode);
  CloseHandle(hProcess);

  return (exitCode == STILL_ACTIVE);
}

void ROS2ProcessManager::refresh_process_cache()
{
  RCLCPP_DEBUG(*logger_, "Refreshing process cache...");

  std::unordered_map<int, ROS2ProcessInfo> new_processes;
  auto monitored_processes_copy(monitored_processes_);

  // 使用工具帮助库遍历进程
  HANDLE hSnapshot = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
  if (hSnapshot == INVALID_HANDLE_VALUE) {
    return;
  }

  PROCESSENTRY32 pe;
  pe.dwSize = sizeof(PROCESSENTRY32);

  if (!Process32First(hSnapshot, &pe)) {
    CloseHandle(hSnapshot);
    return;
  }

  do {
    pid_t pid = pe.th32ProcessID;

    auto cmdline = read_cmd_line(pid);
    auto comm = read_comm(pid);

    if (cmdline.empty()) {
      continue;
    }

    ROS2ProcessInfo info{};

    auto const & [is_monitored, process_name] =
      is_monitored_process(cmdline, monitored_processes_copy);

    if (is_monitored && (!process_name.empty())) {
      info.is_monitored = true;
      info.name = process_name;
      info.node_names = get_node_names(cmdline);
    } else if (is_ros2_process(cmdline, comm)) {
      info.is_monitored = false;
      info.node_names = get_node_names(cmdline);
      if (info.node_names.size() == 1) {
        info.name = info.node_names[0];
      } else {
        info.name = get_process_name(cmdline, comm);
      }
    } else {
      continue;
    }

    info.pid = pid;
    info.last_update = clock_.now();
    new_processes[pid] = info;
  } while (Process32Next(hSnapshot, &pe));

  CloseHandle(hSnapshot);

  // 原子操作更新进程列表
  while (refresh_flag_.test_and_set(std::memory_order_acquire)) {
    std::this_thread::yield();
  }

  processes_ = std::move(new_processes);
  refresh_flag_.clear();
}

#elif __linux__  // Linux implementation

std::string ROS2ProcessManager::read_cmd_line(pid_t pid) const
{
  std::string cmdline;

  std::string cmdline_path = "/proc/" + std::to_string(pid);
  cmdline_path.append("/cmdline");
  std::ifstream cmdline_file(cmdline_path);

  if (cmdline_file) {
    std::getline(cmdline_file, cmdline);
  }
  std::replace(cmdline.begin(), cmdline.end(), '\0', ' ');
  if (cmdline.size() and *cmdline.rbegin() == ' ') {
    cmdline.pop_back();
  }

  return cmdline;
}

std::string ROS2ProcessManager::read_comm(pid_t pid) const
{
  std::string comm;

  std::string comm_path = "/proc/" + std::to_string(pid);
  comm_path.append("/comm");
  std::ifstream comm_file(comm_path);

  if (comm_file) {
    std::getline(comm_file, comm);
  }

  return comm;
}

bool ROS2ProcessManager::is_process_alive(int pid) const { return kill(pid, 0) == 0; }

void ROS2ProcessManager::refresh_process_cache()
{
  RS_DEBUG(nh_, "Refreshing process cache...");
  last_refresh_time_ = SystemTime();

  std::unordered_map<int, ROS2ProcessInfo> new_processes;
  auto monitored_processes_copy(monitored_processes_);

  for (auto const & entry : fs::directory_iterator("/proc")) {
    std::string name = entry.path().filename().string();

    pid_t pid = static_cast<pid_t>(parse_uint64_in_string(name));
    if (pid == 0) {
      continue;
    }

    auto cmdline(read_cmd_line(pid));
    auto comm(read_comm(pid));

    if (cmdline.empty()) {
      continue;
    }

    ROS2ProcessInfo info{};

    auto const & [is_monitored, process_name] =
      is_monitored_process(cmdline, monitored_processes_copy);
    if (is_monitored && (!process_name.empty())) {
      info.is_monitored = true;
      info.name = process_name;
      info.node_names = get_node_names(cmdline);
    } else if (is_ros2_process(cmdline, comm)) {
      info.is_monitored = false;
      info.node_names = get_node_names(cmdline);
      if (info.node_names.size() == 1) {
        info.name = info.node_names[0];
      } else {
        info.name = get_process_name(cmdline, comm);
      }
    } else {
      continue;
    }

    info.pid = pid;
    info.last_update = SystemTime();
    new_processes[pid] = info;
  }

  while (refresh_flag_.test_and_set(std::memory_order_acquire)) {
    std::this_thread::yield();
  }

  processes_ = std::move(new_processes);

  refresh_flag_.clear();
}

#endif

bool ROS2ProcessManager::is_ros2_process(
  std::string const & cmdline, std::string const & comm) const
{
  if (process_identifiers_cmdline_.empty() && process_identifiers_comm_.empty()) {
    return false;
  }

  for (auto const & idenitfier : process_identifiers_comm_) {
    if (comm == idenitfier) {
      return true;
    }
  }

  for (const auto & identifier : process_identifiers_cmdline_) {
    if (cmdline.find(identifier) != std::string::npos) {
      return true;
    }
  }

  return false;
}

std::string ROS2ProcessManager::get_process_name(
  std::string const & cmdline, std::string const & comm) const
{
  size_t pos = cmdline.find(comm);
  std::string temp{};

  if (pos != std::string::npos) {
    return cmdline.substr(pos, max_process_name_length_);
  }

  return cmdline.substr(0, max_process_name_length_);
}

std::vector<std::string> ROS2ProcessManager::get_node_names(std::string const & cmdline) const
{
  char const * kNodeKeyword = "__node:=";

  std::vector<std::string> node_names{};

  auto parts(split_string(cmdline, ' '));
  for (auto const & part : parts) {
    if (part.find(kNodeKeyword) != std::string_view::npos) {
      auto chopped(split_string(part, '='));
      if (chopped.size() == 2) {
        node_names.emplace_back(chopped[1]);
      }
    }
  }

  return node_names;
}

std::pair<bool, std::string> ROS2ProcessManager::is_monitored_process(
  std::string const & cmdline,
  std::vector<ROS2ProcessManager::MonitoredProcess> & monitored_processes) const
{
  std::pair<bool, std::string> result{false, {}};

  if (monitored_processes.empty()) {
    return result;
  }

  for (size_t i = 0, size = monitored_processes.size(); i < size; ++i) {
    auto const & mp = monitored_processes[i];
    if (cmdline.find(mp.keyword) != std::string::npos) {
      result.first = true;
      result.second = std::move(mp.name);

      std::swap(monitored_processes[i], monitored_processes[size - 1]);
      monitored_processes.pop_back();

      return result;
    }
  }
  return result;
}

}  // namespace robosense::rs_monitor
