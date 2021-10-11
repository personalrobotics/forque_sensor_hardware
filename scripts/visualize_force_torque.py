#!/usr/bin/env python
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import WrenchStamped
from matplotlib.animation import FuncAnimation


class Visualizer:
    """
    This class contains the callback to the forque's F/T sensor and uses
    matplotlib's FuncAnimation class to update the visualization of 
    those readings in real time. It displays 6 separate lineplots, one 
    for each of force/torque and x/y/z. 

    TODO: Create a launchfile that opens both this rosnode and rosbag's 
    the messages, as a convenience.
    """
    def __init__(self, init_ylim=(-5,5)):
        self.fig, self.axes = plt.subplots(nrows=2, ncols=3, sharex='all', sharey='row',figsize=(15,8))

        self.lines = []
        for i in range(len(self.axes)):
            for j in range(len(self.axes[i])):
                self.lines.append(self.axes[i][j].plot([], [], 'b-')[0])
                self.axes[i][j].grid('both')
                self.axes[i][j].xaxis.set_tick_params(labelbottom=True)
                self.axes[i][j].yaxis.set_tick_params(labelbottom=True)
        plt.subplots_adjust(wspace=0.3)

        self.x_data, self.y_data = [] , [[] for i in range(len(self.lines))]

        self.start_timestamp = None
        self.init_ylim = init_ylim

    def plot_init(self):
        self.fig.suptitle("Force / Torque Sensor Readings For the Forque")

        self.axes[0][0].set_ylabel('Force X')
        self.axes[0][1].set_ylabel('Force Y')
        self.axes[0][2].set_ylabel('Force Z')
        self.axes[1][0].set_ylabel('Torque X')
        self.axes[1][1].set_ylabel('Torque Y')
        self.axes[1][2].set_ylabel('Torque Z')

        self.axes[0][0].set_xlabel('Time (secs)')
        self.axes[0][1].set_xlabel('Time (secs)')
        self.axes[0][2].set_xlabel('Time (secs)')
        self.axes[1][0].set_xlabel('Time (secs)')
        self.axes[1][1].set_xlabel('Time (secs)')
        self.axes[1][2].set_xlabel('Time (secs)')

        return self.lines 

    def ft_callback(self, msg):
        if self.start_timestamp is None:
            self.start_timestamp = msg.header.stamp

        # Get the data
        t = (msg.header.stamp - self.start_timestamp).to_sec()
        force_x = msg.wrench.force.x
        force_y = msg.wrench.force.y
        force_z = msg.wrench.force.z
        torque_x = msg.wrench.torque.x
        torque_y = msg.wrench.torque.y
        torque_z = msg.wrench.torque.z
        y = [force_x,force_y,force_z,torque_x,torque_y,torque_z]

        # Update the x-axis
        self.x_data.append(t)
        xlim = self.axes[0][0].get_xlim()[1]
        if t > xlim:
            self.axes[0][0].set_xlim(0, t)

        for i in range(len(y)):
            self.y_data[i].append(y[i])

        for i in range(len(self.axes)):
            ylim = self.axes[i][0].get_ylim()
            update_ylim = False
            for j in range(i*3,i*3+3):
                if y[j] < ylim[0]:
                    ylim = (y[j], ylim[1])
                    update_ylim = True
                if y[j] > ylim[1]:
                    ylim = (ylim[0], y[j])
                    update_ylim = True
            if update_ylim:
                self.axes[i][0].set_ylim(*ylim)
    
    def update_plot(self, frame):
        for i in range(len(self.lines)):
            self.lines[i].set_data(self.x_data, self.y_data[i])
        return self.lines


if __name__ == "__main__":
    rospy.init_node('visualize_force_torque')
    vis = Visualizer()
    sub = rospy.Subscriber('/forque/forqueSensor', WrenchStamped, vis.ft_callback)

    ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
    plt.show(block=True) 