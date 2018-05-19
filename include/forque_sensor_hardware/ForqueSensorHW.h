#ifndef FORCE_TORQUE_SENSOR_HW_H_
#define FORCE_TORQUE_SENSOR_HW_H_

#include <iostream>
#include <memory>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/robot_hw.h>
#include <pr_hardware_interfaces/TriggerableInterface.h>
#include <pr_hardware_interfaces/TriggerState.h>
#include <netft_rdt_driver/netft_rdt_driver.h>


namespace forqueSensorHW {

/// The Hardware Interface for the Force/Torque sensor of the Forque.
/// Responsible for connecting, reading data, and calibrating the sensor.
/// Reponds to Trigger signals to start a calibration.
class ForqueSensorHW : public ::hardware_interface::RobotHW {
  
public:

  /// Constructor
  /// \param[in] sensorName Name of the sensor. Data will be published to this topic.
  /// \param[in] frameId Name of the sensor's frame.
  /// \param[in] address IP address of the sensor.
  ForqueSensorHW(std::string sensorName, std::string frameId, std::string address);

  /// Connects to the sensor.
  /// \return True if the connection was established successfully.
  bool connect(bool simulation = false);

  /// Reads from the sensor, handles the data and more.
  /// Needs to be called regularly from  the main loop.
  void update();

private:
  std::string sensorName;
  std::string frameId;
  std::string address;

  /// Interface of the netft sensor device
  std::unique_ptr<netft_rdt_driver::NetFTRDTDriver> netft;

  /// Stores the current force.
  std::array<double, 3> force;

  /// Stores the current torque.
  std::array<double, 3> torque;
  
  // A bunch of variables to handle bias calculation
  bool shouldCollectBiasData = false;
  std::array<double, 6> bias;
  const int totalBiasCollectionSteps = 400;
  int biasCollectionStep = 0;

  /// Kicks off bias calculation and marks end
  pr_hardware_interfaces::TriggerState biasState;

  hardware_interface::ForceTorqueSensorInterface forceTorqueInterface;
  pr_hardware_interfaces::TriggerableInterface biasTriggerInterface; 
};

}

#endif
