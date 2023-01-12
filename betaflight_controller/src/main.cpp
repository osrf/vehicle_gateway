#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "SocketBetaflight.hpp"

using std::placeholders::_1;

#define SIMULATOR_MAX_RC_CHANNELS 16
#define SIMULATOR_MAX_PWM_CHANNELS 16

typedef struct {
    double timestamp;                   // in seconds
    uint16_t channels[SIMULATOR_MAX_RC_CHANNELS]; // (1000-2000) channel values
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
      if(socketRC.init())
      {
        std::cerr << "Not able to open socket" << '\n';
      }

      subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
      rc_packet pkt;
      pkt.timestamp =  msg->header.stamp.sec + msg->header.stamp.nanosec / (1000 * 1000 * 1000);

      pkt.channels[0] = ((msg->axes[3] + 1) / 2 * 1000) + 1000; // roll
      pkt.channels[1] = ((msg->axes[2] + 1) / 2 * 1000) + 1000; // pitch
      pkt.channels[2] = ((msg->axes[1] + 1) / 2 * 1000) + 1000; // yaw
      pkt.channels[3] = ((msg->axes[0] + 1) / 2 * 1000) + 1000; // throtlle
      pkt.channels[4] = (msg->buttons[0] * 1000) + 1000; // throtlle;
      pkt.channels[5] = 1000;
      pkt.channels[6] = 1000;
      pkt.channels[7] = 1000;
      pkt.channels[8] = 1000;
      pkt.channels[9] = 1000;
      pkt.channels[10] = 1000;
      pkt.channels[11] = 1000;
      pkt.channels[12] = 1000;
      pkt.channels[13] = 1000;
      pkt.channels[14] = 1000;
      pkt.channels[15] = 1000;

      std::cerr << "pkt.channels[0] " << pkt.channels[0] << '\n';
      std::cerr << "pkt.channels[1] " << pkt.channels[1] << '\n';
      std::cerr << "pkt.channels[2] " << pkt.channels[2] << '\n';
      std::cerr << "pkt.channels[3] " << pkt.channels[3] << '\n';
      std::cerr << "pkt.channels[4] " << pkt.channels[4] << '\n';
      std::cerr << "pkt.channels[5] " << pkt.channels[5] << '\n';
      std::cerr << "pkt.channels[6] " << pkt.channels[6] << '\n';
      std::cerr << "pkt.channels[7] " << pkt.channels[7] << '\n';
      std::cerr << "---------------------------" << '\n';

      int sentSize = socketRC.udpSend(&pkt, sizeof(rc_packet));
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
