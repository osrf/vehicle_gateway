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

#include <chrono>
#include <iostream>
#include <thread>

#include <pluginlib/class_loader.hpp>
#include <vehicle_gateway/vehicle_gateway.hpp>
#include <vehicle_gateway_betaflight/vehicle_gateway_betaflight.hpp>
using namespace std::chrono_literals;

int main(int argc, const char ** argv)
{
  pluginlib::ClassLoader<vehicle_gateway::VehicleGateway> loader(
    "vehicle_gateway", "vehicle_gateway::VehicleGateway");

  // TODO(anyone): retrieve the plugin name from a ROS 2 param in the launch file
  try {
    std::shared_ptr<vehicle_gateway::VehicleGateway> gateway = loader.createSharedInstance(
      "vehicle_gateway_betaflight::VehicleGatewayBetaflight");
    std::cerr << "Initializing VehicleGatewayBetaflight" << '\n';
    gateway->init(argc, argv);
    int count = 0;
    while (gateway->get_arming_state() != vehicle_gateway::ARMING_STATE::ARMED) {
      if (!gateway->ctbr(0, 0, 0, -1)) {
        std::cerr << "Error sending RC" << '\n';
      }
      if (count == 1) {
        gateway->arm();
      }
      if (count == 30) {
        gateway->disarm();
        count = 0;
      }
      count++;
      std::this_thread::sleep_for(50ms);
      std::cout << "Trying to arm vehicle " << gateway->get_arming_state() << '\n';
    }

    std::cout << "Vehicle is armed" << '\n';

    while (1) {
      std::cerr << "gateway->get_arming_state() " <<
        static_cast<int>(gateway->get_arming_state()) << '\n';
      std::this_thread::sleep_for(50ms);
      if (!gateway->ctbr(0, 0, 0, 0.1)) {
        std::cerr << "Error sending RC" << '\n';
      }
    }
    gateway->destroy();
  } catch (pluginlib::PluginlibException & ex) {
    printf("The plugin failed to load: %s\n", ex.what());
  }
  return 0;
}
