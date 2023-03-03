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

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "SocketBetaflight.hpp"

using std::placeholders::_1;
#define SIMULATOR_MAX_RC_CHANNELS 16
#define SIMULATOR_MAX_PWM_CHANNELS 16

typedef struct
{
  double timestamp;                     // in seconds
  uint16_t channels[SIMULATOR_MAX_RC_CHANNELS];   // (1000-2000) channel values
} rc_packet;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    std::string listen_addr("127.0.0.1");
    uint32_t fdm_port_out = 9004;

    socketRC = SocketBetaflight(listen_addr, fdm_port_out, false);
    if (socketRC.init()) {
      std::cerr << "Not able to open socket" << '\n';
    }

    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    rc_packet pkt;
    pkt.timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec / (1000 * 1000 * 1000);

    pkt.channels[0] = ((-1 * msg->axes[2] + 1) / 2 * 1000) + 1000;   // roll
    pkt.channels[1] = ((msg->axes[3] + 1) / 2 * 1000) + 1000;   // pitch
    pkt.channels[2] = ((msg->axes[1] + 1) / 2 * 1000) + 1000;   // yaw
    pkt.channels[3] = ((msg->axes[0] + 1) / 2 * 1000) + 1000;   // throtlle
    pkt.channels[4] = (msg->buttons[0] * 1000) + 1000;   // aux1;
    pkt.channels[5] = (msg->buttons[1] * 1000) + 1000;   // aux2;
    pkt.channels[6] = (msg->buttons[2] * 1000) + 1000;   // aux3;
    pkt.channels[7] = (msg->buttons[3] * 1000) + 1000;   // aux4;
    pkt.channels[8] = 1000;
    pkt.channels[9] = 1000;
    pkt.channels[10] = 1000;
    pkt.channels[11] = 1000;
    pkt.channels[12] = 1000;
    pkt.channels[13] = 1000;
    pkt.channels[14] = 1000;
    pkt.channels[15] = 1000;

    socketRC.udpSend(&pkt, sizeof(rc_packet));
  }
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  SocketBetaflight socketRC;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
