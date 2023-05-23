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
#include "linux_cpu_system_measurement.hpp"

#include <string>
#include <vector>

LinuxCPUSystemMeasurement::LinuxCPUSystemMeasurement()
: numProcessors_(0)
{
  initCPUProcess();
  makeReading();
}

void LinuxCPUSystemMeasurement::initCPUProcess()
{
  std::string line;
  std::ifstream myfile(std::string("/proc/cpuinfo"));
  if (myfile.is_open()) {
    while (std::getline(myfile, line) ) {
      if (line.find("processor") != std::string::npos) {
        this->numProcessors_++;
      }
    }
  }
  myfile.close();
}

size_t LinuxCPUSystemMeasurement::GetIdleTime(const CPUData & e)
{
  return e.times[S_IDLE] +
         e.times[S_IOWAIT];
}
size_t LinuxCPUSystemMeasurement::GetActiveTime(const CPUData & e)
{
  return e.times[S_USER] +
         e.times[S_NICE] +
         e.times[S_SYSTEM] +
         e.times[S_IRQ] +
         e.times[S_SOFTIRQ] +
         e.times[S_STEAL] +
         e.times[S_GUEST] +
         e.times[S_GUEST_NICE];
}

void LinuxCPUSystemMeasurement::makeReading()
{
  ReadStatsCPU(entries1_);
}

float LinuxCPUSystemMeasurement::getCPUSystemCurrentlyUsed()
{
  ReadStatsCPU(entries2_);

  const size_t NUM_ENTRIES = entries1_.size();

  double total = 0;

  for (size_t i = 0; i < NUM_ENTRIES; ++i) {
    const CPUData & e1 = entries1_[i];
    const CPUData & e2 = entries2_[i];

    const float ACTIVE_TIME = static_cast<float>(GetActiveTime(e2) - GetActiveTime(e1));
    const float IDLE_TIME = static_cast<float>(GetIdleTime(e2) - GetIdleTime(e1));
    const float TOTAL_TIME = ACTIVE_TIME + IDLE_TIME;

    if (TOTAL_TIME > 0.0) {
      total += (100.f * ACTIVE_TIME / TOTAL_TIME);
    }
  }

  entries1_ = entries2_;

  return total / this->numProcessors_;
}

void LinuxCPUSystemMeasurement::ReadStatsCPU(std::vector<CPUData> & entries)
{
  entries.clear();
  std::ifstream fileStat("/proc/stat");

  std::string line;

  const std::string STR_CPU("cpu");
  const std::size_t LEN_STR_CPU = STR_CPU.size();

  while (std::getline(fileStat, line)) {
    // cpu stats line found
    if (!line.compare(0, LEN_STR_CPU, STR_CPU)) {
      std::istringstream ss(line);

      // read cpu label
      std::string label;
      ss >> label;

      if (label.size() > LEN_STR_CPU) {
        label.erase(0, LEN_STR_CPU);
      } else {
        continue;
      }

      // store entry
      entries.emplace_back(CPUData());
      CPUData & entry = entries.back();
      entry.cpu = label;

      // read times
      for (int i = 0; i < NUM_CPU_STATES; ++i) {
        ss >> entry.times[i];
      }
    }
  }
}
