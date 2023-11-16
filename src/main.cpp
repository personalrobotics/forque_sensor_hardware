// Copyright 2023 Personal Robotics Lab, University of Washington
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
// Author: Ethan K. Gordon

#include <bits/stdc++.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "forque_sensor_hardware/wireless_ft.h"

// ROS
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmw/qos_profiles.h"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;
using namespace forque_sensor_hardware;

/* Wireless F/T Node */
class WirelessFTNode : public rclcpp::Node {
public:
  WirelessFTNode(std::shared_ptr<WirelessFT> wft) : Node("wireless_ft") {
    /** Both the timer and service are in separate mutually exclusive
     * callback groups so that the service can be called while the
     * timer is still publishing.
     */
    mTimerCallbackGroup = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    mKeepAliveCallbackGroup = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    mServiceCallbackGroup = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    mWFT = wft;
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};

    // KeepAlive Time Parameter
    param_desc.name = "keepalive_s";
    param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
    param_desc.description = "Rate to check alive F/T sensing, Default 0.5s";
    param_desc.read_only = true;
    declare_parameter("keepalive_s", 0.5, param_desc);

    param_desc.integer_range.push_back(rcl_interfaces::msg::IntegerRange());

    // Rate Parameter
    param_desc.name = "rate";
    param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
    param_desc.description = "F/T Packet transmit rate (Hz), 5-4k, Default 100";
    param_desc.read_only = false;
    param_desc.integer_range[0].from_value = WFT_MIN_RATE;
    param_desc.integer_range[0].to_value = WFT_MAX_RATE;
    param_desc.integer_range[0].step = 1;
    declare_parameter("rate", 50, param_desc);
    mRate = 50;

    // Oversample Parameter
    param_desc.name = "oversample";
    param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
    param_desc.description =
        "F/T Samples per packet, rate*oversample <= 4000, Default 16";
    param_desc.read_only = false;
    param_desc.integer_range[0].from_value = 1;
    param_desc.integer_range[0].to_value = WFT_MAX_RATE;
    param_desc.integer_range[0].step = 1;
    declare_parameter("oversample", 2, param_desc);
    mOversample = 2;

    // Counts Parameters
    param_desc.name = "countsPerN";
    param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
    param_desc.description = "Force Counts Per Newton, Default 1000000";
    param_desc.read_only = false;
    param_desc.integer_range[0].from_value = 1;
    param_desc.integer_range[0].to_value = INT_MAX;
    param_desc.integer_range[0].step = 0; // continuous
    declare_parameter("countsPerN", 1000000, param_desc);

    param_desc.name = "countsPerNm";
    param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
    param_desc.description = "Torque Counts Per Newton-meter, Default 1000000";
    param_desc.read_only = false;
    param_desc.integer_range[0].from_value = 1;
    param_desc.integer_range[0].to_value = INT_MAX;
    param_desc.integer_range[0].step = 0; // continuous
    declare_parameter("countsPerNm", 1000000, param_desc);

    // Force/Torque Frame Parameter
    param_desc.name = "frame";
    param_desc.type = rclcpp::ParameterType::PARAMETER_STRING;
    param_desc.description =
        "TF Frame ID for WrenchStampled, Default 'forque_frame'";
    param_desc.read_only = false;
    declare_parameter("frame", "forque_frame", param_desc);

    // Host Parameter
    param_desc.name = "host";
    param_desc.type = rclcpp::ParameterType::PARAMETER_STRING;
    param_desc.description = "IP or Hostname of F/T Sensor";
    param_desc.read_only = true;
    declare_parameter("host", "ft-sensor", param_desc);

    // Port Parameters
    param_desc.name = "tcpport";
    param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
    param_desc.description = "TCP Port for Telnet, Default 23";
    param_desc.read_only = true;
    declare_parameter("tcpport", DEFAULT_TELNET_PORT, param_desc);

    param_desc.name = "udpport";
    param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
    param_desc.description = "UDP Port for Packet Commands, Default 49152";
    param_desc.read_only = true;
    declare_parameter("udpport", DEFAULT_UDP_PORT, param_desc);

    mCallbackHandle = add_on_set_parameters_callback(std::bind(
        &WirelessFTNode::parametersCallback, this, std::placeholders::_1));
  }

  bool init() {
    // Get Parameters
    auto host = get_parameter("host").get_parameter_value().get<std::string>();
    auto tcpport = get_parameter("tcpport").get_parameter_value().get<int>();
    auto udpport = get_parameter("udpport").get_parameter_value().get<int>();
    auto keepalive_s = std::chrono::duration<double>(
        get_parameter("keepalive_s").get_parameter_value().get<double>());

    // Start up WFT
    if (!mWFT->telnetConnect(host, tcpport)) {
      RCLCPP_ERROR(get_logger(), "Cannot connect to F/T telnet.");
      return false;
    }
    if (!mWFT->udpConfigure(host, udpport)) {
      mWFT->telnetDisconnect();
      RCLCPP_ERROR(get_logger(), "Cannot connect to F/T UDP.");
      return false;
    }

    // Initially no bias (requires service call)
    mWFT->setBias(false);

    // Set Initial Rate
    auto rate = get_parameter("rate").get_parameter_value().get<int>();
    auto oversample =
        get_parameter("oversample").get_parameter_value().get<int>();
    if (!mWFT->setRate(rate, oversample)) {
      RCLCPP_WARN(get_logger(),
                  "Provided rate/oversample failed, reverting to default.");
      rate = 50;
      oversample = 2;
      if (!mWFT->setRate(rate, oversample)) {
        RCLCPP_ERROR(get_logger(), "Cannot set rate");
        return false;
      }
    }
    mRate = rate;
    mOversample = oversample;

    // Set up publishers
    for (int i = 0; i < NUMBER_OF_TRANSDUCERS; i++) {
      mPublishers.push_back(create_publisher<geometry_msgs::msg::WrenchStamped>(
          string_format("~/ftSensor%d", i + 1),
          rclcpp::QoS(1).best_effort().durability_volatile()));
    }

    // Start WFT UDP Streaming
    mWFT->udpStartStreaming();

    // Timer for Polling / Publishing
    // First Init Only
    if (mFirstInit) {
      mFirstInit = false;

      mTimer = this->create_wall_timer(
          1ms, std::bind(&WirelessFTNode::timer_callback, this),
          mTimerCallbackGroup);
      mKeepAlive = this->create_wall_timer(
          keepalive_s, std::bind(&WirelessFTNode::keep_alive_callback, this),
          mKeepAliveCallbackGroup);

      // Service for Bias
      mService = create_service<std_srvs::srv::SetBool>(
          "~/set_bias",
          std::bind(&WirelessFTNode::bias_callback, this, std::placeholders::_1,
                    std::placeholders::_2),
          rmw_qos_profile_services_default, mServiceCallbackGroup);
    }

    RCLCPP_INFO(get_logger(), "Initialization Successful");
    mIsAlive = true;
    return true;
  }

private:
  // For setting new rate / oversample
  rcl_interfaces::msg::SetParametersResult
  parametersCallback(const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    bool changeRate = false;
    int provisionalRate = mRate;
    int provisionalOversample = mOversample;
    for (const auto &parameter : parameters) {
      if (parameter.get_name() == "rate" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        changeRate = true;
        provisionalRate = parameter.get_parameter_value().get<int>();
      } else if (parameter.get_name() == "oversample" &&
                 parameter.get_type() ==
                     rclcpp::ParameterType::PARAMETER_INTEGER) {
        changeRate = true;
        provisionalOversample = parameter.get_parameter_value().get<int>();
      }
    }
    if (changeRate) {
      if (mWFT->setRate(provisionalRate, provisionalOversample)) {
        RCLCPP_INFO(get_logger(), "Rate changed successfully: %d, %d", mRate,
                    mOversample);
        mRate = provisionalRate;
        mOversample = provisionalOversample;
      } else {
        result.successful = false;
        result.reason = "Cannot change rate";
      }
    }
    return result;
  }

  // Check if alive, and if not, re-init
  void keep_alive_callback() {
    if (!mIsAlive) {
      RCLCPP_WARN(get_logger(), "KeepAlive Failed, re-init");
      // Disconnect and Re-connect
      mWFT->udpClose();
      mWFT->telnetDisconnect();

      this->init();
      return;
    }
    mIsAlive = false;
  }

  // For reading from F/T Sensor and publishing wrench
  void timer_callback() {
    auto packet = mWFT->readDataPacket();
    if (!packet.valid) {
      //RCLCPP_WARN(get_logger(), "Skipping Invalid Packet");
      return;
    }
    mIsAlive = true;

    // Convert to ROS Timestamp
    std::int32_t secs =
        std::chrono::time_point_cast<std::chrono::seconds>(packet.timestamp)
            .time_since_epoch()
            .count();
    std::uint32_t nsecs =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            packet.timestamp -
            std::chrono::floor<std::chrono::seconds>(packet.timestamp))
            .count();

    // Get Parameters
    std::string frame =
        get_parameter("frame").get_parameter_value().get<std::string>();
    double ncounts =
        (double)(get_parameter("countsPerN").get_parameter_value().get<int>());
    double nmcounts =
        (double)(get_parameter("countsPerNm").get_parameter_value().get<int>());

    for (int i = 0; i < NUMBER_OF_TRANSDUCERS; i++) {
      if (!packet.transducer_present[i])
        continue;
      auto msg = geometry_msgs::msg::WrenchStamped();

      // Message Header
      msg.header.stamp.sec = secs;
      msg.header.stamp.nanosec = nsecs;
      msg.header.frame_id = frame;

      // Convert to N / Nm
      msg.wrench.force.x = (double)(packet.counts[i][0]) / ncounts;
      msg.wrench.force.y = (double)(packet.counts[i][1]) / ncounts;
      msg.wrench.force.z = (double)(packet.counts[i][2]) / ncounts;
      msg.wrench.torque.x = (double)(packet.counts[i][3]) / nmcounts;
      msg.wrench.torque.y = (double)(packet.counts[i][4]) / nmcounts;
      msg.wrench.torque.z = (double)(packet.counts[i][5]) / nmcounts;

      // Publish
      mPublishers[i]->publish(msg);
    }
  }

  void
  bias_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    RCLCPP_INFO(get_logger(), "Started setting bias...");

    response->success = true;
    response->message = "re-taring success";

    if (!mWFT->setBias(request->data)) {
      response->success = false;
      response->message = "error in setBias";
    }
    RCLCPP_INFO(get_logger(), "...finished setting bias!");
  }

  // Parameters
  int mRate;
  int mOversample;

  // Keep Alive
  std::atomic<bool> mIsAlive = true;
  bool mFirstInit = true;

  // ROS Objects
  OnSetParametersCallbackHandle::SharedPtr mCallbackHandle;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr>
      mPublishers;
  rclcpp::TimerBase::SharedPtr mTimer, mKeepAlive;
  rclcpp::CallbackGroup::SharedPtr mTimerCallbackGroup, mKeepAliveCallbackGroup;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr mService;
  rclcpp::CallbackGroup::SharedPtr mServiceCallbackGroup;

  // WFT
  std::shared_ptr<WirelessFT> mWFT;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Init Wireless F/T and Node
  auto wft = std::make_shared<WirelessFT>();
  auto node = std::make_shared<WirelessFTNode>(wft);
  if (!node->init()) {
    rclcpp::shutdown();
    return -1;
  }

  /** Use the MultiThreadedExecutor to enable the node to continue
   * publishing F/T readings during the setBias service call.
   * Use 2 threads for the 2 different callbacks (timer to publish,
   * service to setBias)
   */
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                    2);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();

  // Cleanup Wireless F/T
  wft->udpStopStreaming();
  wft->udpClose();
  wft->telnetDisconnect();
  return 0;
}
