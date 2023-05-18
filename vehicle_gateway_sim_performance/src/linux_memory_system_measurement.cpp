// Copyright 2023 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "linux_memory_system_measurement.hpp"
#include <string>
#include <vector>
#include <limits>

LinuxMemorySystemMeasurement::LinuxMemorySystemMeasurement() {}

double LinuxMemorySystemMeasurement::getTotalMemorySystem()
{
  std::string token;
  std::ifstream file("/proc/meminfo");
  while (file >> token) {
    if (token == "MemTotal:") {
      uint64_t mem;
      if (file >> mem) {
        return mem / 1024.0;       // Mb
      } else {
        return 0;
      }
    }
    // ignore rest of the line
    file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }
  return 0;
}

double LinuxMemorySystemMeasurement::getFreeMemorySystem()
{
  std::string token;
  std::ifstream file("/proc/meminfo");
  while (file >> token) {
    if (token == "MemFree:") {
      uint64_t mem;
      if (file >> mem) {
        return mem / 1024.0;       // Mb
      } else {
        return 0;
      }
    }
    // ignore rest of the line
    file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }
  return 0;
}

double LinuxMemorySystemMeasurement::getAvailableMemorySystem()
{
  std::string token;
  std::ifstream file("/proc/meminfo");
  while (file >> token) {
    if (token == "MemAvailable:") {
      uint64_t mem;
      if (file >> mem) {
        return mem / 1024.0;       // Mb
      } else {
        return 0;
      }
    }
    // ignore rest of the line
    file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }
  return 0;
}
