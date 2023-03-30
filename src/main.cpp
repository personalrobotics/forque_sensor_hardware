#include <bits/stdc++.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "forque_sensor_hardware/wireless_ft.h"

// ROS
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/WrenchStamped.h>
#include <pr_control_msgs/TriggerAction.h>
#include <ros/ros.h>

using namespace std::chrono_literals;
using namespace forque_sensor_hardware;

/* Wireless F/T Node */
class WirelessFTNode {
public:
  WirelessFTNode(std::shared_ptr<WirelessFT> wft)
      : nh_("~"), as_(nh_, "~/bias_controller/trigger",
                      std::bind(&WirelessFTNode::bias_callback, this,
                                std::placeholders::_1),
                      false) {
    mWFT = wft;
    mSequence = 0;
  }

  bool init() {
    // Get Parameters
    std::string host = nh_.param<std::string>("~host", "ft-sensor");

    int tcpport = nh_.param<int>("~tcpport", 23);

    int udpport = nh_.param<int>("~udpport", 49152);

    // Start up WFT
    if (!mWFT->telnetConnect(host, tcpport)) {
      ROS_ERROR("Cannot connect to F/T telnet.");
      return false;
    }
    if (!mWFT->udpConfigure(host, udpport)) {
      mWFT->telnetDisconnect();
      ROS_ERROR("Cannot connect to F/T UDP.");
      return false;
    }

    // Initial bias
    mWFT->setBias(true);

    // Set Initial Rate
    int rate = nh_.param<int>("~rate", 100);
    int oversample = nh_.param<int>("~oversample", 16);

    if (!mWFT->setRate(rate, oversample)) {
      ROS_WARN("Provided rate/oversample failed, reverting to default.");
      rate = 100;
      oversample = 16;
      if (!mWFT->setRate(rate, oversample)) {
        ROS_ERROR("Cannot set rate");
        return false;
      }
    }

    // Set up publisher (first transducer only)
    mPublisher =
        nh_.advertise<geometry_msgs::WrenchStamped>("~forqueSensor", 1);

    // Start WFT UDP Streaming
    mWFT->udpStartStreaming();

    // Timer for Polling / Publishing
    mTimer = nh_.createTimer(ros::Duration(0.001),
                             std::bind(&WirelessFTNode::timer_callback, this));

    ROS_INFO("Initialization Successful");
    return true;
  }

private:
  // For reading from F/T Sensor and publishing wrench
  void timer_callback() {
    auto packet = mWFT->readDataPacket();
    if (!packet.valid) {
      ROS_WARN("Skipping Invalid Packet");
      return;
    }

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
    std::string frame = nh_.param<std::string>("~frame", "forque_frame");
    double ncounts = (double)(nh_.param<int>("~countsPerN", 1000000));
    double nmcounts = (double)(nh_.param<int>("~countsPerNm", 1000000));

    int i = 0;
    if (!packet.transducer_present[i])
      return;
    auto msg = geometry_msgs::WrenchStamped();

    // Message Header
    msg.header.seq = mSequence;
    msg.header.stamp = ros::Time(secs, nsecs);
    msg.header.frame_id = frame;

    // Convert to N / Nm
    msg.wrench.force.x = (double)(packet.counts[i][0]) / ncounts;
    msg.wrench.force.y = (double)(packet.counts[i][1]) / ncounts;
    msg.wrench.force.z = (double)(packet.counts[i][2]) / ncounts;
    msg.wrench.torque.x = (double)(packet.counts[i][3]) / nmcounts;
    msg.wrench.torque.y = (double)(packet.counts[i][4]) / nmcounts;
    msg.wrench.torque.z = (double)(packet.counts[i][5]) / nmcounts;

    // Publish
    mPublisher.publish(msg);
    mSequence++;
  }

  void bias_callback(const pr_control_msgs::TriggerGoalConstPtr &goal) {
    pr_control_msgs::TriggerResult response;
    response.success = true;
    response.message = "re-taring success";

    if (!mWFT->setBias(true)) {
      response.success = false;
      response.message = "error in setBias";
    }
    as_.setSucceeded(response);
  }

  // ROS Objects
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<pr_control_msgs::TriggerAction> as_;
  ros::Publisher mPublisher;
  ros::Timer mTimer;
  std::uint32_t mSequence;

  // WFT
  std::shared_ptr<WirelessFT> mWFT;
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "forque");

  // Init Wireless F/T and Node
  auto wft = std::make_shared<WirelessFT>();
  WirelessFTNode node(wft);
  if (!node.init()) {
    return -1;
  }

  ros::spin();

  // Cleanup Wireless F/T
  wft->udpStopStreaming();
  wft->udpClose();
  wft->telnetDisconnect();
  return 0;
}
