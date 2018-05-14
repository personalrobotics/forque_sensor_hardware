#include "forque_hardware_interface/TestRobotHW.h"

TestRobotHW::TestRobotHW() {

  hardware_interface::InterfaceManager::registerInterface(&forceTorqueInterface);
  hardware_interface::InterfaceManager::registerInterface(&biasTriggerInterface);
}