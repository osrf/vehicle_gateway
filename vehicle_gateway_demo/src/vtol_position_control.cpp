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

using namespace std;

class VehicleGatewayCpp
{
public:
  VehicleGatewayCpp()
  {
    this->loader_ = std::make_shared<pluginlib::ClassLoader<vehicle_gateway::VehicleGateway>>(
      "vehicle_gateway", "vehicle_gateway::VehicleGateway");
    this->gateway_ = this->loader_->createSharedInstance("vehicle_gateway_px4::VehicleGatewayPX4");
    this->gateway_->init(0, nullptr);
  }

  void destroy()
  {
    if (this->gateway_) {
      this->gateway_->destroy();
      this->gateway_ = nullptr;
    }
    if (this->loader_) {
      this->loader_->unloadLibraryForClass(
        "vehicle_gateway_px4::VehicleGatewayPX4");
      this->loader_ = nullptr;
    }
  }

  std::shared_ptr<vehicle_gateway::VehicleGateway> gateway_;

private:
  std::shared_ptr<pluginlib::ClassLoader<vehicle_gateway::VehicleGateway>> loader_;
};


int main(int argc, const char * argv[])
{
  rclcpp::init(argc, argv);

  const auto vg = std::make_shared<VehicleGatewayCpp>();
  const float TARGET_ATTITUDE = 30.0f;

  cout << "Arming..." << endl;
  vg->gateway_->arm_sync();
  sleep(2);

  const auto home_position = vg->gateway_->get_latlon();
  if (home_position.size() != 3) {
    throw std::runtime_error("home_position should have three elements: lat, lon, alt");
  }
  cout << "Home altitude: " << home_position[2] << endl;

  cout << "Takeoff!" << endl;
  vg->gateway_->takeoff();
  sleep(1);

  vg->gateway_->go_to_latlon_sync(
    home_position[0], home_position[1],
    home_position[2] + TARGET_ATTITUDE);

  vg->gateway_->transition_to_fw_sync();

  cout << "Begin transitioning to Offboard control..." << endl;
  vg->gateway_->transition_to_offboard_sync();

  cout << "Enabled position controller" << endl;
  auto target_north = 200.0,
    target_east = 0.0;

  cout << "Flying to first waypoint..." << endl;
  vg->gateway_->offboard_mode_go_to_local_setpoint_sync(
    target_north, target_east, -TARGET_ATTITUDE,
    numeric_limits<float>::quiet_NaN(), 15);
  cout << "Flying to second waypoint..." << endl;
  vg->gateway_->offboard_mode_go_to_local_setpoint_sync(
    -target_north, target_east, -TARGET_ATTITUDE,
    numeric_limits<float>::quiet_NaN(), 20);
  cout << "Flying home..." << endl;
  vg->gateway_->offboard_mode_go_to_local_setpoint_sync(
    0, 0, -TARGET_ATTITUDE,
    numeric_limits<float>::quiet_NaN(), 20);

  cout << "Switching back to hold mode..." << endl;
  vg->gateway_->set_onboard_mode();

  cout << "Transitioning to multicopter..." << endl;
  vg->gateway_->transition_to_mc_sync();
  cout << "VTOL state: " << vg->gateway_->get_vtol_state() << endl;
  sleep(1);

  vg->gateway_->go_to_latlon_sync(
    home_position[0], home_position[1],
    home_position[2] + TARGET_ATTITUDE);

  cout << "Landing..." << endl;
  vg->gateway_->land();

  cout << "Disarming..." << endl;
  vg->gateway_->disarm_sync();

  vg->destroy();
  cout << "Demo complete." << endl;

  rclcpp::shutdown();
  return 0;
}
