
#include "forque_hardware_interface/ForceTorqueSensorHW.h"

ForceTorqueSensorHW::ForceTorqueSensorHW(std::string sensorName, std::string frameId, std::string ipAddress) :
    mSensorName(sensorName),
    mFrameId(frameId),
    mAddress(ipAddress) {

  mForce.fill(0);
  mTorque.fill(0);
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

    mForce[0] = data.wrench.force.x;
    mForce[1] = data.wrench.force.y;
    mForce[2] = data.wrench.force.z;
    mTorque[0] = data.wrench.torque.x;
    mTorque[1] = data.wrench.torque.y;
    mTorque[2] = data.wrench.torque.z;
  }
}

void ForceTorqueSensorHW::registerHandle(hardware_interface::ForceTorqueSensorInterface& interface) {
  interface.registerHandle(hardware_interface::ForceTorqueSensorHandle(
    mSensorName, mFrameId, &mForce[0], &mTorque[0]
  ));
}