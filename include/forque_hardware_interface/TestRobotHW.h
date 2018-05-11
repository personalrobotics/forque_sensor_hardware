#ifndef TEST_ROBOT_HW_H_
#define TEST_ROBOT_HW_H_


#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

class TestRobotHW : public ::hardware_interface::RobotHW {

public:

  TestRobotHW();

  hardware_interface::ForceTorqueSensorInterface forceTorqueInterface;

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  double cmd[2];
  double pos[2];
  double vel[2];
  double eff[2];

};

#endif