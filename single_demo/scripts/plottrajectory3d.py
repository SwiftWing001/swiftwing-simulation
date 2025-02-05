#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import os
import sys

dir_mytest = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
dir_mylib = dir_mytest + "/gvf_lib"
sys.path.insert(0, dir_mylib)
from gvf_lib.PolynomialTraj import PolynomialTraj3Order

class UAVTrajectoryPlotter:
    """Class to plot UAV trajectory in 3D space using ROS data.

    This class listens to the /mavros/local_position/pose topic to
    receive the UAV's 3D position data and plots the trajectory in real-time.
    """

    def __init__(self):
        """Initialize the ROS node, subscriber, and plot setup."""
        # Initialize ROS node
        rospy.init_node('uav_trajectory_plotter', anonymous=True)
        
        # Initialize trajectory data lists
        self.x_data = []
        self.y_data = []
        self.z_data = []
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        
        # Subscribe to the /mavros/local_position/pose topic
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        
        # Set up interactive plotting with matplotlib
        plt.ion()  # Enable interactive mode
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlabel('X Position')
        self.ax.set_ylabel('Y Position')
        self.ax.set_zlabel('Z Position')
        # self.get_poly_trajectory()
        # Display the plot
        plt.show()

    def pose_callback(self, data):
        """Callback function to process position data from the ROS topic.

        This function extracts the position data (x, y, z) from the received
        PoseStamped message and updates the trajectory data lists.
        """
        x = data.pose.position.x
        y = data.pose.position.y
        z = data.pose.position.z
        
        # Update trajectory data
        self.x_data.append(x)
        self.y_data.append(y)
        self.z_data.append(z)
        self.x = x
        self.y = y
        self.z = z

    def start_plotting(self):
        """Real-time plotting of the UAV trajectory."""
        # Clear the previous plot
        self.ax.cla()
        
        # set range of x,y axis
        # self.ax.set_xlim3d(-150, 150)
        # self.ax.set_ylim3d(-150, 150)

        # Dynamically set the Z-axis limits  
        self.ax.set_zlim(0, 70)
        # h_max = max(self.x_data)
        # self.ax.set_zlim(0, h_max+20)

        # Get desire trajectory
        self.get_circle_trajectory()

        # Plot the trajectory
        self.ax.plot(self.x_traj, self.y_traj, self.z_traj, color='blue')
        self.ax.plot(self.x_data, self.y_data, self.z_data, label='Trajectory', marker='o', markersize=2, color='black')
        
        # Plot the current position as a red dot
        self.ax.scatter(self.x, self.y, self.z, color='red', s=100)
        
        # Pause to update the plot
        plt.pause(0.05)
        plt.draw()

    def get_circle_trajectory(self):
        """get circle trajectory points"""
        theta = np.linspace(0, 2 * np.pi, 100)
        self.x_traj = 100 * np.cos(theta)
        self.y_traj = 100 * np.sin(theta)
        self.z_traj = np.full_like(self.x_traj,50.)


    def get_inclinecircle_trajectory(self):
        """get inclined circle trajectory points"""
        theta = np.linspace(0, 2 * np.pi, 100)
        self.x_traj = 100 * np.cos(theta)
        self.y_traj = 100 * np.sin(theta)
        self.z_traj = -self.x_traj/10.+150.

    def get_lisa_trajectory(self):
        """Plot a sample circular trajectory (unused in this script)."""
        amplitude = np.array([225., 225., -20.])
        phase = np.array([0.0001, 0.0002, 0.0002])
        offset = np.array([0., np.pi/2., 0.])
        add = np.array([0., 0., 80.])
        angle_list = np.linspace(0., 50000 * np.pi, 300)
        self.x_traj = amplitude[0] * np.cos(phase[0] * angle_list + offset[0])
        self.y_traj = amplitude[1] * np.cos(phase[1] * angle_list + offset[1])
        self.z_traj = amplitude[2] * np.cos(phase[2] * angle_list + offset[2]) + add[2]

    def get_poly_trajectory(self):
        points = np.array([[0., 0., 150.],
                            [400., 0., 150.],
                            [400., 1000., 150.],
                            [800., 1000., 150.],
                            [800., 0., 150.],
                            [1200., 0., 150.],
                            [1200., 2400., 150.],
                            [800., 2400., 150.],
                            [800., 1400., 150.],
                            [400., 1400., 150.],
                            [400., 2400., 150.],
                            [0., 2400., 150.]]) / 4.
        points[:, 2] = 100.

        dimension = 3
        traj = PolynomialTraj3Order(dimension, points, loop_flag=1)
        sample_traj = traj.GetSampleTraj()
        self.x_traj = sample_traj[:,0]
        self.y_traj = sample_traj[:,1]
        self.z_traj = sample_traj[:,2]
        


if __name__ == '__main__':
    # Create an instance of the UAVTrajectoryPlotter class
    plotter = UAVTrajectoryPlotter()

    # Keep the script running and plotting until ROS shuts down
    while not rospy.is_shutdown():
        plotter.start_plotting()
