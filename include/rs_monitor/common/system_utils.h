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

#ifndef RS_MONITOR_COMMON_SYSTEM_UTILS_H
#define RS_MONITOR_COMMON_SYSTEM_UTILS_H

#include <memory>
#include <string>
#include <system_error>

#ifdef _WIN32
#include <windows.h>

namespace robosense::rs_monitor
{

class ScopedHandle
{
public:
  explicit ScopedHandle(HANDLE handle) : handle_(handle) {}
  ~ScopedHandle()
  {
    if (handle_ != INVALID_HANDLE_VALUE && handle_ != nullptr) {
      CloseHandle(handle_);
    }
  }

  HANDLE get() const { return handle_; }

  ScopedHandle(const ScopedHandle &) = delete;
  ScopedHandle & operator=(const ScopedHandle &) = delete;

private:
  HANDLE handle_;
};

class SystemError : public std::system_error
{
public:
  SystemError(const std::string & what)
  : std::system_error(GetLastError(), std::system_category(), what)
  {
  }
};

}  // namespace robosense::rs_monitor

#endif  // _WIN32

#endif  // RS_MONITOR_COMMON_SYSTEM_UTILS_H
