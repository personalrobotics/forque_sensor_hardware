#ifndef TEST_ROBOT_HW_H_
#define TEST_ROBOT_HW_H_


#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/robot_hw.h>

class TestRobotHW : public ::hardware_interface::RobotHW {

public:

  hardware_interface::ForceTorqueSensorInterface forceTorqueInterface;

private:
  

};

#endif