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

#include <functional>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <pluginlib/class_loader.hpp>
#include <vehicle_gateway/vehicle_gateway.hpp>
#include <vehicle_gateway_px4/vehicle_gateway_px4.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

class DroneCircles : public rclcpp::Node
{
public:
  DroneCircles(int argc, const char ** argv)
  : Node("minimal_subscriber"), loader_("vehicle_gateway", "vehicle_gateway::VehicleGateway")
  {
    this->gateway_ = this->loader_.createSharedInstance(
      "vehicle_gateway_px4::VehicleGatewayPX4");
    RCLCPP_INFO(this->get_logger(), "Initializing VehicleGatewayPX4");
    this->gateway_->init(0, nullptr);
    this->thread_loop_ = std::thread(&DroneCircles::loop, this);
  }

  void loop()
  {
    int offboard_setpoint_counter_ = 0;

    while (offboard_setpoint_counter_ < 6) {
      offboard_setpoint_counter_++;
      std::this_thread::sleep_for(100ms);
      this->gateway_->set_offboard_control_mode(vehicle_gateway::POSITION);
      this->gateway_->set_local_position_setpoint(0, 0, -3, 0);
      if (offboard_setpoint_counter_ == 5) {
        while (true) {
          RCLCPP_INFO(this->get_logger(), "Try to set offboard mode and arm vehicle");
          this->gateway_->set_offboard_mode();
          this->gateway_->arm();
          std::this_thread::sleep_for(200ms);
          if ((this->gateway_->get_flight_mode() == vehicle_gateway::FLIGHT_MODE::OFFBOARD &&
            this->gateway_->get_arming_state() == vehicle_gateway::ARMING_STATE::ARMED) ||
            this->stopped_)
          {
            RCLCPP_INFO(this->get_logger(), "Vehicle is in Offboard mode");
            RCLCPP_INFO(this->get_logger(), "Vehicle is armed");
            break;
          }
          std::this_thread::sleep_for(200ms);
          this->gateway_->disarm();
          std::this_thread::sleep_for(200ms);
        }
      }
    }

    while (this->gateway_->get_altitude() > -2.5 && !this->stopped_) {
      std::this_thread::sleep_for(50ms);
      this->gateway_->set_offboard_control_mode(vehicle_gateway::POSITION);
      this->gateway_->set_local_position_setpoint(0, 0, -5, 0);
      RCLCPP_INFO(this->get_logger(), "altitude %.2f", this->gateway_->get_altitude());
    }

    std::chrono::time_point<std::chrono::system_clock> last_update_time =
      std::chrono::system_clock::now();

    while (!this->stopped_) {
      this->gateway_->set_offboard_control_mode(vehicle_gateway::POSITION);
      this->gateway_->set_local_position_setpoint(
        this->radius * cos(this->theta),
        this->radius * sin(this->theta),
        -3,
        0);

      std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();

      auto diff =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_update_time);
      this->theta = this->theta + this->omega * diff.count() / 1000.0;

      last_update_time = now;
      std::this_thread::sleep_for(50ms);
    }
    RCLCPP_INFO(this->get_logger(), "Vehicle end");
  }

  ~DroneCircles()
  {
    this->stopped_ = true;
    this->thread_loop_.join();
    this->gateway_->destroy();
  }

private:
  pluginlib::ClassLoader<vehicle_gateway::VehicleGateway> loader_;
  std::shared_ptr<vehicle_gateway::VehicleGateway> gateway_;
  std::thread thread_loop_;
  bool stopped_ = false;

  float theta = 0.0;
  float radius = 4.0;
  float omega = 0.5;
};

int main(int argc, const char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneCircles>(argc, argv));
  rclcpp::shutdown();
  return 0;
}
