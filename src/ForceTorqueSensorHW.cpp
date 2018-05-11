
#include "forque_hardware_interface/ForceTorqueSensorHW.h"

ForceTorqueSensorHW::ForceTorqueSensorHW(std::string sensorName, std::string frameId, std::string ipAddress) :
    mSensorName(sensorName),
    mFrameId(frameId),
    mAddress(ipAddress) {

  mForce.fill(0);
  mTorque.fill(0);
  bias.fill(0);
  biasCollectionStep = 0;
}

void ForceTorqueSensorHW::connect(bool simulate) {
  if (simulate) {
    mForce[0] = 12.23;
  } else {
    netft = std::unique_ptr<netft_rdt_driver::NetFTRDTDriver>( new netft_rdt_driver::NetFTRDTDriver(mAddress));
  }
}

void ForceTorqueSensorHW::update() {
  

  if (netft) {
    geometry_msgs::WrenchStamped data;
    netft->getData(data);

    if (biasCollectionStep < totalBiasCollectionSteps) {

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
      }
    } else {
      std::string msg = "FT: " + std::to_string(data.wrench.force.x) + " " + std::to_string(data.wrench.force.y) + " " + std::to_string(data.wrench.force.z) + " " + std::to_string(data.wrench.torque.x) + " " + std::to_string(data.wrench.torque.y) + " " + std::to_string(data.wrench.torque.z);
      //std::cout << msg << std::endl;
      mForce[0] = data.wrench.force.x - bias[0];
      mForce[1] = data.wrench.force.y - bias[1];
      mForce[2] = data.wrench.force.z - bias[2];
      mTorque[0] = data.wrench.torque.x - bias[3];
      mTorque[1] = data.wrench.torque.y - bias[4];
      mTorque[2] = data.wrench.torque.z - bias[5];
    }
  }
}

void ForceTorqueSensorHW::registerHandle(hardware_interface::ForceTorqueSensorInterface& interface) {
  hardware_interface::ForceTorqueSensorHandle forqueSensorHandle(mSensorName, mFrameId, &mForce[0], &mTorque[0]);
  interface.registerHandle(forqueSensorHandle);

}