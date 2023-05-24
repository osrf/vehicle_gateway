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

#include <iostream>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vehicle_gateway_px4/vehicle_gateway_px4.hpp>
#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#include <stdlib.h>

std::shared_ptr<pluginlib::ClassLoader<vehicle_gateway::VehicleGateway>> loader_;
std::mutex mutex;

class VehicleGatewayCpp
{
public:
  explicit VehicleGatewayCpp(unsigned int _vehicle_id)
  {
    this->gateway_ = loader_->createSharedInstance("vehicle_gateway_px4::VehicleGatewayPX4");
    this->gateway_->set_vehicle_id(_vehicle_id);
    this->gateway_->init(0, nullptr);
  }

  void destroy()
  {
    std::cerr << "destroy" << '\n';
    if (this->gateway_) {
      this->gateway_->destroy();
      this->gateway_ = nullptr;
    }
    if (loader_) {
      loader_->unloadLibraryForClass(
        "vehicle_gateway_px4::VehicleGatewayPX4");
      loader_ = nullptr;
    }
  }

  std::shared_ptr<vehicle_gateway::VehicleGateway> gateway_;
};

void position_control(int vehicle_id)
{
  std::cerr << "vehicle_id " << vehicle_id << '\n';

  mutex.lock();
  // This call is not thread safe.
  const auto vg = std::make_shared<VehicleGatewayCpp>(vehicle_id);
  mutex.unlock();

  const float TARGET_ATTITUDE = 30.0f + vehicle_id * 5;

  std::cout << "Arming... " << vehicle_id << std::endl;
  vg->gateway_->arm_sync();
  sleep(2);

  const auto home_position = vg->gateway_->get_latlon();
  if (home_position.size() != 3) {
    throw std::runtime_error("home_position should have three elements: lat, lon, alt");
  }
  std::cout << "Home altitude: " << vehicle_id << " - " << home_position[2] << std::endl;

  std::cout << "Takeoff! " << vehicle_id << std::endl;
  vg->gateway_->takeoff();
  sleep(1);

  vg->gateway_->go_to_latlon_sync(
    home_position[0], home_position[1],
    home_position[2] + TARGET_ATTITUDE);

  vg->gateway_->transition_to_fw_sync();

  std::cout << "Begin transitioning to Offboard control... " << vehicle_id << std::endl;
  vg->gateway_->transition_to_offboard_sync();

  std::cout << "Enabled position controller " << vehicle_id << std::endl;
  auto target_north = 200.0,
    target_east = 0.0;

  std::cout << "Flying to first waypoint... " << vehicle_id << std::endl;
  vg->gateway_->offboard_mode_go_to_local_setpoint_sync(
    target_north, target_east, -TARGET_ATTITUDE,
    std::numeric_limits<float>::quiet_NaN(), 15);
  std::cout << "Flying to second waypoint... " << vehicle_id << std::endl;
  vg->gateway_->offboard_mode_go_to_local_setpoint_sync(
    -target_north, target_east, -TARGET_ATTITUDE,
    std::numeric_limits<float>::quiet_NaN(), 20);
  std::cout << "Flying home... " << vehicle_id << std::endl;
  vg->gateway_->offboard_mode_go_to_local_setpoint_sync(
    0, 0, -TARGET_ATTITUDE,
    std::numeric_limits<float>::quiet_NaN(), 20);

  std::cout << "Switching back to hold mode... " << vehicle_id << std::endl;
  vg->gateway_->set_onboard_mode();

  std::cout << "Transitioning to multicopter... " << vehicle_id << std::endl;
  vg->gateway_->transition_to_mc_sync();
  std::cout << "VTOL state:  " << vehicle_id << " - " << vg->gateway_->get_vtol_state()
            << std::endl;
  sleep(1);

  vg->gateway_->go_to_latlon_sync(
    home_position[0], home_position[1],
    home_position[2] + TARGET_ATTITUDE);

  std::cout << "Landing... " << vehicle_id << std::endl;
  vg->gateway_->land();

  std::cout << "Disarming... " << vehicle_id << std::endl;
  vg->gateway_->disarm_sync();

  vg->destroy();
  std::cout << "Test complete. " << vehicle_id << std::endl;
}

int main(int argc, const char * argv[])
{
  rclcpp::init(argc, argv);

  loader_ = std::make_shared<pluginlib::ClassLoader<vehicle_gateway::VehicleGateway>>(
    "vehicle_gateway", "vehicle_gateway::VehicleGateway");

  const char * ros_domain_id_c = getenv("ROS_DOMAIN_ID");
  int number_of_vehicles = 1;
  int ros_domain_id = 0;
  if (ros_domain_id_c != nullptr) {
    ros_domain_id = atoi(ros_domain_id_c);
  } else {
    const char * number_of_vehicles_c = getenv("NUMBER_OF_VEHICLES");
    if (number_of_vehicles_c != nullptr) {
      number_of_vehicles = atoi(number_of_vehicles_c);
    }
  }

  std::cout << "ROS_DOMAIN_ID " << ros_domain_id << '\n';
  std::cout << "NUMBER_OF_VEHICLES " << number_of_vehicles << '\n';

  std::vector<std::thread> threads;
  for (int i = 0; i < number_of_vehicles; i++) {
    if (ros_domain_id == 0) {
      threads.push_back(std::thread(position_control, i + 1));
    } else {
      threads.push_back(std::thread(position_control, ros_domain_id));
    }
  }
  for (int i = 0; i < number_of_vehicles; i++) {
    threads[i].join();
  }

  rclcpp::shutdown();
  return 0;
}
