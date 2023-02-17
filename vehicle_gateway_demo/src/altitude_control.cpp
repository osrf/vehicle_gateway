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
#include <std_msgs/msg/float64.hpp>
#include <control_toolbox/pid.hpp>

#include <pluginlib/class_loader.hpp>
#include <vehicle_gateway/vehicle_gateway.hpp>
#include <vehicle_gateway_px4/vehicle_gateway_px4.hpp>

#include <aruco_opencv_msgs/msg/marker_detection.hpp>

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
    this->subscription_marker_ = this->create_subscription<aruco_opencv_msgs::msg::MarkerDetection>(
      "/marker_detections", 10,
      std::bind(&MinimalSubscriber::topic_callback, this, _1));

    this->gateway_ = this->loader_.createSharedInstance(
      "vehicle_gateway_px4::VehicleGatewayPX4");
    RCLCPP_INFO(this->get_logger(), "Initializing VehicleGatewayPX4");
    this->gateway_->init(0, nullptr);
    this->thread_loop_ = std::thread(&MinimalSubscriber::loop, this);

    this->x_control_pid_ = control_toolbox::Pid(0.08, 0, 0);
    this->y_control_pid_ = control_toolbox::Pid(0.08, 0, 0);
  }

  void loop()
  {
    int offboard_setpoint_counter_ = 0;

    while (offboard_setpoint_counter_ < 6)
    {
      offboard_setpoint_counter_++;
      std::this_thread::sleep_for(100ms);
      this->gateway_->set_offboard_control_mode(true);
      this->gateway_->set_local_position_setpoint(0, 0, -5);
      if (offboard_setpoint_counter_ == 5)
      {
        // while ((this->gateway_->get_flight_mode() != vehicle_gateway::FLIGHT_MODE::OFFBOARD
        //       || this->gateway_->get_arming_state() != vehicle_gateway::ARMING_STATE::ARMED)
        //       && !this->stopped_)
        // {
          this->gateway_->set_offboard_mode();
          this->gateway_->arm();
          // std::this_thread::sleep_for(10ms);
          // this->gateway_->disarm();
          // std::this_thread::sleep_for(200ms);

        // }
        RCLCPP_INFO(this->get_logger(), "Vehicle is in Offboard mode");
        RCLCPP_INFO(this->get_logger(), "Vehicle is armed");
      }
    }

    while (this->gateway_->get_altitude() > -4.5 && !this->stopped_) {
      std::this_thread::sleep_for(50ms);
      this->gateway_->set_offboard_control_mode(true);
      this->gateway_->set_local_position_setpoint(0, 0, -5);
      RCLCPP_INFO(this->get_logger(), "altitude %.2f", this->gateway_->get_altitude());
    }

    std::chrono::time_point<std::chrono::system_clock> last_update_time =
      std::chrono::system_clock::now();

    while(!this->stopped_)
    {
    //
    //
    //   std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
    //
    //   auto diff =
    //     std::chrono::duration_cast<std::chrono::nanoseconds>(now - last_update_time);
    //   last_update_time = now;
    //
      float x, y, z;
      this->gateway_->get_local_position(x, y, z);
      RCLCPP_INFO(this->get_logger(), "x %.2f \t y: %.2f \t %.2f", x, y, z);

      std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();

      {
        std::lock_guard<std::mutex> lock(mutex_marker);
        if (this->marker_detected_)
        {
          RCLCPP_INFO(this->get_logger(), "x %.2f \t y: %.2f",  this->marker_x_,  this->marker_y_);

          x -= this->marker_x_;
          y -= this->marker_y_;

          this->marker_x_ = 0;
          this->marker_y_ = 0;
          this->marker_detected_ = false;
          last_update_time = now;
        }
      }

      auto diff =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_update_time);

      if (diff.count() > 1000)
      {
        this->gateway_->set_local_position_setpoint(0, 0, -5);
        RCLCPP_INFO(this->get_logger(), "Marker is not visible");

      }
      else
      {
        this->gateway_->set_local_position_setpoint(x, y, -5);
      }
      std::this_thread::sleep_for(50ms);
    }
    RCLCPP_INFO(this->get_logger(), "Vehicle end");

  }

  ~MinimalSubscriber()
  {
    this->stopped_ = true;
    this->thread_loop_.join();
    this->gateway_->destroy();
  }

private:
  void topic_callback(const aruco_opencv_msgs::msg::MarkerDetection & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_marker);

    if (msg.markers.size() > 0)
    {
      auto marker = msg.markers[0];
      this->marker_x_ = marker.pose.position.x;
      this->marker_y_ = marker.pose.position.y;
      this->marker_z_ = marker.pose.position.z;
      this->marker_detected_ = true;
    }
  }

  std::mutex mutex_marker;
  float marker_x_;
  float marker_y_;
  float marker_z_;
  bool marker_detected_{false};

  float x_{0};
  float y_{0};
  float z_{0};
  control_toolbox::Pid x_control_pid_;
  control_toolbox::Pid y_control_pid_;

  pluginlib::ClassLoader<vehicle_gateway::VehicleGateway> loader_;
  std::shared_ptr<vehicle_gateway::VehicleGateway> gateway_;
  rclcpp::Subscription<aruco_opencv_msgs::msg::MarkerDetection>::SharedPtr subscription_marker_;
  std::thread thread_loop_;
  bool stopped_ = false;
};

int main(int argc, const char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>(argc, argv));
  rclcpp::shutdown();
  return 0;
}
