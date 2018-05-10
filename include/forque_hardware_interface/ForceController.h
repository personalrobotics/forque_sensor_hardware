#ifndef FORCE_CONTROLLER_H_
#define FORCE_CONTROLLER_H_


#include <rewd_controllers/MultiInterfaceController.hpp>
#include <rewd_controllers/JointTrajectoryControllerBase.hpp>
#include <hardware_interface/force_torque_sensor_interface.h>


class ForceController final
    : public rewd_controllers::MultiInterfaceController<
                                      hardware_interface::PositionJointInterface,
                                      hardware_interface::VelocityJointInterface,
                                      hardware_interface::EffortJointInterface,
                                      hardware_interface::JointStateInterface,
                                      hardware_interface::ForceTorqueSensorInterface>,
      rewd_controllers::JointTrajectoryControllerBase
{

public:

  bool init(hardware_interface::RobotHW* robot, ros::NodeHandle& n) override;

  void starting(const ros::Time& time) override;

  void stopping(const ros::Time& time) override;

  void update(const ros::Time& time, const ros::Duration& period) override;

protected:

  bool shouldAcceptRequests() override;

  bool shouldStopExecution() override;
};

#endif