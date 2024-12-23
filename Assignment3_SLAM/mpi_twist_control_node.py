#!/usr/bin/env python3
""" MegaPi Controller ROS2 Wrapper"""
#import rospy
import rclpy # replaces rospy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from mpi_control import MegaPiController
import numpy as np


class MegaPiControllerNode(Node):
    def init(self, verbose=True, debug=False):
        super().init('megapi_controller_node')
        self.mpi_ctrl = MegaPiController(port='/dev/ttyUSB0', verbose=verbose)
        self.r = 0.025 # radius of the wheel
        self.lx = 0.055 # half of the distance between front wheel and back wheel
        self.ly = 0.07 # half of the distance between left wheel and right wheel
        self.calibration = 100.0
        self.subscription = self.create_subscription(Twist, '/twist', self.twist_callback, 10)
        self.subscription

    def twist_callback(self, twist_cmd):
#        desired_twist = self.calibration * np.array([[twist_cmd.linear.x], [twist_cmd.linear.y], [twist_cmd.angular.z]])
      #  desired_twist = np.array([[100*twist_cmd.linear.x], [110*twist_cmd.linear.y], [140*twist_cmd.angular.z]])
        desired_twist = np.array([[80*twist_cmd.linear.x], [100*twist_cmd.linear.y], [120*twist_cmd.angular.z]])
        # calculate the jacobian matrix
        jacobian_matrix = np.array([[1, -1, -(self.lx + self.ly)],
                                     [1, 1, (self.lx + self.ly)],
                                     [1, 1, -(self.lx + self.ly)],
                                     [1, -1, (self.lx + self.ly)]]) / self.r
        # calculate the desired wheel velocity
        result = np.dot(jacobian_matrix, desired_twist)

        # send command to each wheel
        print(result)
        self.mpi_ctrl.setFourMotors(int(result[0][0]), int(result[1][0]), int(result[2][0]), int(result[3][0]))

        

if name == "main":
    rclpy.init()
    mpi_ctrl_node = MegaPiControllerNode()
    #rospy.init_node('megapi_controller')
    #rospy.Subscriber('/twist', Twist, mpi_ctrl_node.twist_callback, queue_size=1) 

    
    rclpy.spin(mpi_ctrl_node) # Spin for until shutdown

    # Destroy node and shutdown when done. (Optional, as node would be cleaned up by garbage collection)
    mpi_ctrl_node.destroy_node()
    rclpy.shutdown()