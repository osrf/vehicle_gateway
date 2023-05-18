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

#include <semaphore.h>
#include <chrono>
#include <csignal>
#include <string>
#include "linux_memory_system_measurement.hpp"
#include "linux_cpu_system_measurement.hpp"

static sem_t sentinel;

static void post_sentinel(int signum)
{
  (void)signum;
  const char msg[] = "\nGot signal - shutting down\n";
  (void)!write(STDERR_FILENO, msg, sizeof(msg));
  sem_post(&sentinel);
}

int main(int argc, char * argv[])
{
  float timeout = 60;
  std::ofstream m_os;
  std::string process_pid;
  struct timespec wait_until;

  signal(SIGINT, post_sentinel);

  std::string filename = argv[1];
  m_os.open(filename, std::ofstream::out);
  std::cout << "file_name " << filename << std::endl;
  if (m_os.is_open()) {
    m_os << "T_experiment" << ",system_cpu_usage (%)"
         << ",system virtual memory (Mb)" << std::endl;
  }

  auto start = std::chrono::high_resolution_clock::now();

  LinuxCPUSystemMeasurement linux_cpu_system_measurement;
  LinuxMemorySystemMeasurement linux_memory_system_measurement;

  while (true) {
    double cpu_system_percentage = linux_cpu_system_measurement.getCPUSystemCurrentlyUsed();
    double phy_mem_system_usage = linux_memory_system_measurement.getTotalMemorySystem() -
      linux_memory_system_measurement.getAvailableMemorySystem();

    std::cout << "CPU Usage system: " << cpu_system_percentage << " (%)" << std::endl;
    std::cout << "PhysMemUsedsystem: " << phy_mem_system_usage << " (Mb)" << std::endl;

    clock_gettime(CLOCK_REALTIME, &wait_until);
    wait_until.tv_sec += 1;
    if (sem_timedwait(&sentinel, &wait_until) == 0 || errno != ETIMEDOUT) {
      break;
    }

    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = finish - start;
    float seconds_running = elapsed.count() / 1000;
    std::cout << "------------------------- " << std::endl;

    if (seconds_running > timeout) {
      break;
    }

    if (m_os.is_open()) {
      m_os << seconds_running << ", " << cpu_system_percentage <<
        ", " << phy_mem_system_usage << std::endl;
    }
  }

  return 0;
}
