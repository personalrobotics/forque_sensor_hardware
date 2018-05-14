
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <controller_manager/controller_manager.h>
#include <forque_sensor_hardware/ForqueSensorHW.h>
#include <ros/callback_queue.h>

bool shouldShutdown = false;

void mySigintHandler(int sig) {
  shouldShutdown = true;
}

int main(int argc, char** argv) {
  // Hardcoded values
  std::string address = "192.168.1.1";
  float update_rate_hz = 100;


  // Starting ROS
  ROS_INFO("Starting ROS node.");
  ros::init(argc, argv, "forque_hardware_interface", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  ros::CallbackQueue queue;
  nh.setCallbackQueue(&queue);
  signal(SIGINT, mySigintHandler);


  // Starting other stuff
  ForqueSensorHW forqueSensorHW("forqueSensor", "endEffectorFrameId", address);
  forqueSensorHW.registerHandles();
  
  controller_manager::ControllerManager cm(&forqueSensorHW, nh);
  if (!forqueSensorHW.connect()) {
    return 0;
  }

  ros::AsyncSpinner spinner(0, &queue);
  spinner.start();


  // Update loop
  ros::Rate update_rate(update_rate_hz);
  ros::Time ts = ros::Time::now();
  while (!shouldShutdown)
  {
    forqueSensorHW.update();
    ros::Duration d = ros::Time::now() - ts;
    ts = ros::Time::now();
    cm.update(ts, d);
    update_rate.sleep();
  }


  // wait for a bit, so controller_manager can unload controllers
  ros::Duration shutdownDuration(1.5);
  ros::Time shutdownBegin = ros::Time::now();
  while (ros::Time::now() - shutdownBegin < shutdownDuration) {
    ros::Duration d = ros::Time::now() - ts;
    ts = ros::Time::now();
    cm.update(ts, d);
  }

  spinner.stop();
  return 0;
}
