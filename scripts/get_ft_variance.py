#!/usr/bin/env python
import rospy
from geometry_msgs.msg import WrenchStamped


class GetFTVariance:
    """
    This class subscribes to the force/torque sensor readings, computes the 
    running variance of those readings, and outputs them to terminal.

    A sample usage of the script would be to investigate whether the 
    resting force/torque sensor has a low or high variance, where a 
    high variance may be indicative of a hardware issue.
    """
    def __init__(self):
        self.n = 0

        self.sum_force_x = 0.0
        self.sum_force_y = 0.0
        self.sum_force_z = 0.0
        self.sum_torque_x = 0.0
        self.sum_torque_y = 0.0
        self.sum_torque_z = 0.0

        self.sum_force_x_sq = 0.0
        self.sum_force_y_sq = 0.0
        self.sum_force_z_sq = 0.0
        self.sum_torque_x_sq = 0.0
        self.sum_torque_y_sq = 0.0
        self.sum_torque_z_sq = 0.0

    @staticmethod
    def get_variance(total, total_sq, n):
        return total_sq/n - (total**2.0)/(n**2.0)

    def ft_callback(self, msg):
        self.n += 1

        self.sum_force_x += msg.wrench.force.x
        self.sum_force_y += msg.wrench.force.y
        self.sum_force_z += msg.wrench.force.z
        self.sum_torque_x += msg.wrench.torque.x
        self.sum_torque_y += msg.wrench.torque.y
        self.sum_torque_z += msg.wrench.torque.z

        self.sum_force_x_sq += msg.wrench.force.x**2.0
        self.sum_force_y_sq += msg.wrench.force.y**2.0
        self.sum_force_z_sq += msg.wrench.force.z**2.0
        self.sum_torque_x_sq += msg.wrench.torque.x**2.0
        self.sum_torque_y_sq += msg.wrench.torque.y**2.0
        self.sum_torque_z_sq += msg.wrench.torque.z**2.0

        force_x_var = GetFTVariance.get_variance(
            self.sum_force_x, self.sum_force_x_sq, self.n)
        force_y_var = GetFTVariance.get_variance(
            self.sum_force_y, self.sum_force_y_sq, self.n)
        force_z_var = GetFTVariance.get_variance(
            self.sum_force_z, self.sum_force_z_sq, self.n)
        torque_x_var = GetFTVariance.get_variance(
            self.sum_torque_x, self.sum_torque_x_sq, self.n)
        torque_y_var = GetFTVariance.get_variance(
            self.sum_torque_y, self.sum_torque_y_sq, self.n)
        torque_z_var = GetFTVariance.get_variance(
            self.sum_torque_z, self.sum_torque_z_sq, self.n)

        rospy.loginfo("Force: (%.5e, %.5e, %.5e)\nTorque: (%.5e, %.5e, %.5e)" % (
            force_x_var, force_y_var, force_z_var, 
            torque_x_var, torque_y_var, torque_z_var))



if __name__ == "__main__":
    rospy.init_node('get_ft_variance', log_level=rospy.INFO)
    get_ft_variance = GetFTVariance()
    sub = rospy.Subscriber('/forque/forqueSensor', WrenchStamped, get_ft_variance.ft_callback)

    rospy.spin()