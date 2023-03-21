#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "forque_sensor_hardware/wireless_ft.h"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace forque_sensor_hardware;

/* Wireless F/T Node */
class WirelessFTNode : public rclcpp::Node
{
public:
  WirelessFTNode(std::shared_ptr<WirelessFT> wft) : Node("wireless_ft")
  {
    mWFT = wft;
    mWFT->udpStartStreaming();
    // Timer for Polling
    timer_ = this->create_wall_timer(10ms, std::bind(&WirelessFTNode::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto packet = mWFT->readDataPacket();
    auto latency =
      std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()) -
      packet.timestamp.time_since_epoch();
    RCLCPP_INFO(this->get_logger(), "System Time: %f", std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count());
    RCLCPP_INFO(this->get_logger(), "Device Time: %f", packet.timestamp.time_since_epoch().count());
    RCLCPP_INFO(this->get_logger(), "Valid? '%d'; Latency: '%f'", packet.valid, latency.count());
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<WirelessFT> mWFT;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto logger = rclcpp::get_logger("General");
  RCLCPP_INFO(logger, "Entering Node.");

  // Init Wireless F/T
  auto wft = std::make_shared<WirelessFT>();
  if (!wft->telnetConnect("ada-router")) {
    RCLCPP_ERROR(logger, "Cannot connect to F/T telnet.");
    rclcpp::shutdown();
    return -1;
  }
  if (!wft->udpConfigure("ada-router")) {
    wft->telnetDisconnect();
    RCLCPP_ERROR(logger, "Cannot connect to F/T UDP.");
    rclcpp::shutdown();
    return -1;
  }

  wft->setRate(100, 16);
  wft->setBias(false);

  rclcpp::spin(std::make_shared<WirelessFTNode>(wft));
  rclcpp::shutdown();

  // Cleanup Wireless F/T
  RCLCPP_INFO(logger, "Node Cleanup...");
  wft->udpStopStreaming();
  wft->udpClose();
  wft->telnetDisconnect();
  return 0;
}
