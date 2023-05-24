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
#ifndef LINUX_CPU_SYSTEM_MEASUREMENT_HPP_
#define LINUX_CPU_SYSTEM_MEASUREMENT_HPP_

#include <unistd.h>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <chrono>
#include <thread>
#include <iostream>

enum CPUStates
{
  S_USER = 0,
  S_NICE,
  S_SYSTEM,
  S_IDLE,
  S_IOWAIT,
  S_IRQ,
  S_SOFTIRQ,
  S_STEAL,
  S_GUEST,
  S_GUEST_NICE,
};

const int NUM_CPU_STATES = 10;

typedef struct CPUData
{
  std::string cpu;
  size_t times[NUM_CPU_STATES];
} CPUData;

class LinuxCPUSystemMeasurement
{
public:
  LinuxCPUSystemMeasurement();
  void ReadStatsCPU(std::vector<CPUData> & entries);
  float getCPUSystemCurrentlyUsed();

private:
  size_t GetIdleTime(const CPUData & e);
  size_t GetActiveTime(const CPUData & e);
  void initCPUProcess();
  void makeReading();

  std::vector<CPUData> entries1_;
  std::vector<CPUData> entries2_;
  int numProcessors_;
};

#endif  // LINUX_CPU_SYSTEM_MEASUREMENT_HPP_
