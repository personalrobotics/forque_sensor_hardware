
#include "forque_hardware_interface/ForqueSensorHW.h"

ForqueSensorHW::ForqueSensorHW(std::string sensorName, std::string frameId, std::string ipAddress) :
    mSensorName(sensorName),
    mFrameId(frameId),
    mAddress(ipAddress),
    mBiasState(pr_hardware_interfaces::TRIGGER_IDLE) {

  mForce.fill(0);
  mTorque.fill(0);
  bias.fill(0);
  biasCollectionStep = 0;
}

bool ForqueSensorHW::connect() {
    try {
    netft = std::unique_ptr<netft_rdt_driver::NetFTRDTDriver>( new netft_rdt_driver::NetFTRDTDriver(mAddress));
  } catch (std::runtime_error e) {
    netft = nullptr;
    std::cerr << "Error when starting NetFT device: " << e.what() << std::endl;
    return false;
  }
  return true;
}

void ForqueSensorHW::update() {

  if (mBiasState == pr_hardware_interfaces::TRIGGER_REQUESTED) {
    shouldCollectBiasData = true;
    mBiasState = pr_hardware_interfaces::TRIGGER_PENDING;
  }

  if (netft) {
    geometry_msgs::WrenchStamped data;
    netft->getData(data);

    if (shouldCollectBiasData && biasCollectionStep < totalBiasCollectionSteps) {

      bias[0] += data.wrench.force.x;
      bias[1] += data.wrench.force.y;
      bias[2] += data.wrench.force.z;
      bias[3] += data.wrench.torque.x;
      bias[4] += data.wrench.torque.y;
      bias[5] += data.wrench.torque.z;

      biasCollectionStep++;
      if (biasCollectionStep == totalBiasCollectionSteps) {
        for (int i=0; i<6; i++) {
          bias[i] = bias[i] / totalBiasCollectionSteps;
        }
        shouldCollectBiasData = false;
        biasCollectionStep = 0;
        mBiasState = pr_hardware_interfaces::TRIGGER_IDLE;
      }
    } else {
      std::string msg = "FT: " + std::to_string(data.wrench.force.x) + " " + std::to_string(data.wrench.force.y) + " " + std::to_string(data.wrench.force.z) + " " + std::to_string(data.wrench.torque.x) + " " + std::to_string(data.wrench.torque.y) + " " + std::to_string(data.wrench.torque.z);
      //ROS_INFO(msg.c_str());
      mForce[0] = data.wrench.force.x - bias[0];
      mForce[1] = data.wrench.force.y - bias[1];
      mForce[2] = data.wrench.force.z - bias[2];
      mTorque[0] = data.wrench.torque.x - bias[3];
      mTorque[1] = data.wrench.torque.y - bias[4];
      mTorque[2] = data.wrench.torque.z - bias[5];
    }
  }
}

void ForqueSensorHW::registerHandles() {
  hardware_interface::ForceTorqueSensorHandle forqueSensorHandle(mSensorName, mFrameId, &mForce[0], &mTorque[0]);
  forceTorqueInterface.registerHandle(forqueSensorHandle);
  hardware_interface::InterfaceManager::registerInterface(&forceTorqueInterface);

  pr_hardware_interfaces::TriggerableHandle biasHandle("/biasTrigger", &mBiasState);
  biasTriggerInterface.registerHandle(biasHandle);
  hardware_interface::InterfaceManager::registerInterface(&biasTriggerInterface);
}
