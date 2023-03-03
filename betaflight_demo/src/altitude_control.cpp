// Copyright 2016 Open Source Robotics Foundation, Inc.
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
#include <std_msgs/msg/float64.hpp>
#include <control_toolbox/pid.hpp>

#include <pluginlib/class_loader.hpp>
#include <vehicle_gateway/vehicle_gateway.hpp>
#include <vehicle_gateway_betaflight/vehicle_gateway_betaflight.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

template<typename T>
inline T clamp(T _v, T _min, T _max)
{
  return std::max(std::min(_v, _max), _min);
}

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber(int argc, const char ** argv)
  : Node("minimal_subscriber"), loader_("vehicle_gateway", "vehicle_gateway::VehicleGateway")
  {
    this->subscription_ = this->create_subscription<std_msgs::msg::Float64>(
      "global_position/rel_alt", rclcpp::SensorDataQoS(),
      std::bind(&MinimalSubscriber::topic_callback, this, _1));

    this->gateway_ = this->loader_.createSharedInstance(
      "vehicle_gateway_betaflight::VehicleGatewayBetaflight");
    RCLCPP_INFO(this->get_logger(), "Initializing VehicleGatewayBetaflight");
    this->gateway_->init(0, nullptr);
    this->thread_loop_ = std::thread(&MinimalSubscriber::loop, this);

    this->altitude_control_pid_ = control_toolbox::Pid(0.08, 0.02, 0.002);
    this->altitude_control_pid_.reset();
  }

  void loop()
  {
    int count = 0;
    while (this->gateway_->get_arming_state() != vehicle_gateway::ARMING_STATE::ARMED) {
      if (!this->gateway_->ctbr(0, 0, 0, -1)) {
        RCLCPP_ERROR(this->get_logger(), "Error sending RC");
      }
      if (count == 1) {
        this->gateway_->arm();
      }
      if (count == 100) {
        this->gateway_->disarm();
      }
      if (count == 150) {
        count = 0;
      }
      count++;
      std::this_thread::sleep_for(50ms);
      RCLCPP_INFO(
        this->get_logger(), "Trying to arm vehicle %d",
        this->gateway_->get_arming_state());
    }

    RCLCPP_INFO(this->get_logger(), "Vehicle is armed");


    std::chrono::time_point<std::chrono::system_clock> last_update_time =
      std::chrono::system_clock::now();

    while (1) {
      std::this_thread::sleep_for(50ms);

      double altitude;
      {
        std::lock_guard<std::mutex> lock(mutex_altitude);
        altitude = this->altitude_;
      }
      std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();

      auto nanoseconds =
        std::chrono::duration_cast<std::chrono::nanoseconds>(now - last_update_time);

      last_update_time = now;

      double error = this->desired_altitude_ - altitude;
      double target_vel =
        this->altitude_control_pid_.computeCommand(
        error,
        nanoseconds.count());
      target_vel = clamp(target_vel, -1.0, 1.0);
      RCLCPP_INFO(
        this->get_logger(), "target_vel %.2f desired alt: %.2f real alt: %.2f ",
        target_vel, this->desired_altitude_, altitude);
      if (!this->gateway_->ctbr(0, 0, 0, target_vel)) {
        RCLCPP_ERROR(this->get_logger(), "Error sending RC");
      }
    }
  }

  ~MinimalSubscriber()
  {
    this->thread_loop_.join();
    this->gateway_->destroy();
  }

private:
  void topic_callback(const std_msgs::msg::Float64 & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_altitude);
    this->altitude_ = msg.data;
    RCLCPP_INFO(this->get_logger(), "I heard: '%.2f'", msg.data);
  }

  pluginlib::ClassLoader<vehicle_gateway::VehicleGateway> loader_;
  std::shared_ptr<vehicle_gateway::VehicleGateway> gateway_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
  std::thread thread_loop_;

  double altitude_{0};
  double desired_altitude_{5};
  std::mutex mutex_altitude;
  control_toolbox::Pid altitude_control_pid_;
};

int main(int argc, const char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>(argc, argv));
  rclcpp::shutdown();
  return 0;
}
