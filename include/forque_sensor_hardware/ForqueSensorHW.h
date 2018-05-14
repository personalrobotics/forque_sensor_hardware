#ifndef FORCE_TORQUE_SENSOR_HW_H_
#define FORCE_TORQUE_SENSOR_HW_H_

#include <iostream>
#include <memory>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/robot_hw.h>
#include <pr_hardware_interfaces/TriggerableInterface.h>
#include <pr_hardware_interfaces/TriggerState.h>
#include <forque_sensor_hardware/netft/netft_rdt_driver.h>
#include <forque_sensor_hardware/netft/netft_rdt_bias.h>


class ForqueSensorHW : public ::hardware_interface::RobotHW {
  
public:

  ForqueSensorHW(std::string sensorName, std::string frameId, std::string address);
  bool connect();
  void update();

  void registerHandles();

private:
  std::string mSensorName;
  std::string mFrameId;
  std::string mAddress;
  std::unique_ptr<netft_rdt_driver::NetFTRDTDriver> netft;

  std::array<double, 3> mForce;
  std::array<double, 3> mTorque;
  
  bool shouldCollectBiasData = false;
  std::array<double, 6> bias;
  const int totalBiasCollectionSteps = 400;
  int biasCollectionStep;

  pr_hardware_interfaces::TriggerState mBiasState;
  hardware_interface::ForceTorqueSensorInterface forceTorqueInterface;
  pr_hardware_interfaces::TriggerableInterface biasTriggerInterface; 
};

#endif
