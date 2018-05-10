
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <controller_manager/controller_manager.h>
#include <forque_hardware_interface/ForceController.h>
#include <forque_hardware_interface/TestRobotHW.h>
#include <forque_hardware_interface/ForceTorqueSensorHW.h>



int main(int argc, char** argv) {
  std::string address = "192.168.1.1";
  float update_rate_hz = 100;


  ROS_INFO("Starting ROS node.");
  ros::init(argc, argv, "forque_hardware_interface");
  ros::NodeHandle nh("~");





  TestRobotHW testRobotHW;
  ForceTorqueSensorHW forceTorqueSensor("forqueSensor", "endEffectorFrameId", address);
  forceTorqueSensor.registerHandle(testRobotHW.forceTorqueInterface);
  
  controller_manager::ControllerManager cm(&testRobotHW);

  forceTorqueSensor.connect(false);

  ros::Rate update_rate(update_rate_hz);

/*
  while (ros::ok()) {
    forceTorqueSensor.update();

    ROS_INFO("spin");
    ros::spinOnce();
    update_rate.sleep();
  }*/

  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::Time ts = ros::Time::now();

  ros::Rate rate(50);
  while (ros::ok())
  {
    forceTorqueSensor.update();
    ros::Duration d = ros::Time::now() - ts;
    ts = ros::Time::now();
    //robot.read();
    cm.update(ts, d);
    //robot.write();
    update_rate.sleep();
  }

  spinner.stop();
  return 0;
}