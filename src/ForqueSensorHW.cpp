
#include "forque_sensor_hardware/ForqueSensorHW.h"

namespace forqueSensorHW {

//==============================================================================
ForqueSensorHW::ForqueSensorHW(std::string sensorName, std::string frameId,
                               std::string ipAddress)
    : sensorName(sensorName), frameId(frameId), address(ipAddress),
      biasState(pr_hardware_interfaces::TRIGGER_IDLE) {

  force.fill(0);
  torque.fill(0);
  bias.fill(0);

  // Register Handles
  hardware_interface::ForceTorqueSensorHandle forqueSensorHandle(
      sensorName, frameId, &force[0], &torque[0]);
  forceTorqueInterface.registerHandle(forqueSensorHandle);
  hardware_interface::InterfaceManager::registerInterface(
      &forceTorqueInterface);

  pr_hardware_interfaces::TriggerableHandle biasHandle("/biasTrigger",
                                                       &biasState);
  biasTriggerInterface.registerHandle(biasHandle);
  hardware_interface::InterfaceManager::registerInterface(
      &biasTriggerInterface);
}

//==============================================================================
bool ForqueSensorHW::connect(bool simulation) {
  if (simulation) {
    ROS_WARN("NetFT is only simulated");
    netft = nullptr;
  } else {
    try {
      netft = std::unique_ptr<netft_rdt_driver::NetFTRDTDriver>(
          new netft_rdt_driver::NetFTRDTDriver(address));
    } catch (std::runtime_error e) {
      netft = nullptr;
      std::cerr << "Error when starting NetFT device: " << e.what() << std::endl;
      return false;
    }
  }

  // start bias collection
  shouldCollectBiasData = true;
  biasState = pr_hardware_interfaces::TRIGGER_PENDING;
  ROS_INFO("Starting F/T sensor calibration");
  return true;
}

//==============================================================================
bool ForqueSensorHW::update() {

  if (biasState == pr_hardware_interfaces::TRIGGER_REQUESTED) {
    shouldCollectBiasData = true;
    biasState = pr_hardware_interfaces::TRIGGER_PENDING;
    ROS_INFO("Starting F/T sensor calibration");
  }

  if (netft && netft->waitForNewData()) {
    geometry_msgs::WrenchStamped data;
    netft->getData(data);

    if (shouldCollectBiasData &&
        biasCollectionStep < totalBiasCollectionSteps) {

      bias[0] += data.wrench.force.x;
      bias[1] += data.wrench.force.y;
      bias[2] += data.wrench.force.z;
      bias[3] += data.wrench.torque.x;
      bias[4] += data.wrench.torque.y;
      bias[5] += data.wrench.torque.z;

      biasCollectionStep++;
      if (biasCollectionStep == totalBiasCollectionSteps) {
        for (int i = 0; i < 6; i++) {
          bias[i] = bias[i] / totalBiasCollectionSteps;
        }
        shouldCollectBiasData = false;
        biasCollectionStep = 0;
        biasState = pr_hardware_interfaces::TRIGGER_IDLE;
        ROS_INFO("Finished F/T sensor calibration");
      }
    } else {
      force[0] = data.wrench.force.x - bias[0];
      force[1] = data.wrench.force.y - bias[1];
      force[2] = data.wrench.force.z - bias[2];
      torque[0] = data.wrench.torque.x - bias[3];
      torque[1] = data.wrench.torque.y - bias[4];
      torque[2] = data.wrench.torque.z - bias[5];
    }
    return true;
  } else {
    ROS_ERROR("NetFT Connection Lost.");
    return false;
  }
}
}
