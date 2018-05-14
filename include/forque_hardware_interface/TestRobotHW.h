#ifndef TEST_ROBOT_HW_H_
#define TEST_ROBOT_HW_H_


#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <pr_hardware_interfaces/TriggerableInterface.h>

class TestRobotHW : public ::hardware_interface::RobotHW {

public:

  TestRobotHW();

  hardware_interface::ForceTorqueSensorInterface forceTorqueInterface;
  pr_hardware_interfaces::TriggerableInterface biasTriggerInterface;

private:

};

#endif