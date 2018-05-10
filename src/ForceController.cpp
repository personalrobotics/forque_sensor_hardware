
#include <forque_hardware_interface/ForceController.h>


bool ForceController::init(hardware_interface::RobotHW* robot, ros::NodeHandle& n) {
  return false;
}

void ForceController::starting(const ros::Time& time) {
  
}

void ForceController::stopping(const ros::Time& time) {
  
}

void ForceController::update(const ros::Time& time, const ros::Duration& period) {
  
}

bool ForceController::shouldAcceptRequests() {
  return false;
}

bool ForceController::shouldStopExecution() {
  return false;
}