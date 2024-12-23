#!/usr/bin/env python3
import sys
#import rospy

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
import numpy as np
from april_detection import PoseSubscriber
"""
The class of the pid controller.
"""
class PIDcontroller(Node):
    def __init__(self, Kp, Ki, Kd):
        super().__init__('PID_Controller_NodePub')
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target = None
        self.I = np.array([0.0,0.0,0.0])
        self.lastError = np.array([0.0,0.0,0.0])
        self.timestep = 0.1
        self.maximumValue = 0.1
        self.publisher_ = self.create_publisher(Twist, '/twist', 10)

    def setTarget(self, targetx, targety, targetw):
        """
        set the target pose.
        """
        self.I = np.array([0.0,0.0,0.0])
        self.lastError = np.array([0.0,0.0,0.0])
        self.target = np.array([targetx, targety, targetw])

    def setTarget(self, state):
        """
        set the target pose.
        """
        self.I = np.array([0.0,0.0,0.0])
        self.lastError = np.array([0.0,0.0,0.0])
        self.target = np.array(state)

    def getError(self, currentState, targetState):
        """
        return the different between two states
        """
        result = targetState - currentState
        result[2] = (result[2] + np.pi) % (2 * np.pi) - np.pi
        return result

    def setMaximumUpdate(self, mv):
        """
        set maximum velocity for stability.
        """
        self.maximumValue = mv

    def update(self, currentState):
        """
        calculate the update value on the state based on the error between curre
nt state and target state with PID.
        """
        e = self.getError(currentState, self.target)

        P = self.Kp * e
        self.I = self.I + self.Ki * e * self.timestep
        I = self.I
        D = self.Kd * (e - self.lastError)
        result = P + I + D

        self.lastError = e

        # scale down the twist if its norm is more than the maximum value. 
        resultNorm = np.linalg.norm(result)
        if(resultNorm > self.maximumValue):
            result = (result / resultNorm) * self.maximumValue
            self.I = 0.0

        return result

def genTwistMsg(desired_twist):
    """
    Convert the twist to twist msg.
    """
    twist_msg = Twist()
    twist_msg.linear.x = desired_twist[0]
    twist_msg.linear.y = desired_twist[1]
    twist_msg.linear.z = 0.0
    twist_msg.angular.x = 0.0
    twist_msg.angular.y = 0.0
    twist_msg.angular.z = desired_twist[2]
    return twist_msg
# def get_april_tag(rclpy,currentState):
#     pose_subscriber = PoseSubscriber(currentState)
#     rclpy.spin_once(pose_subscriber)
#     calib_state = pose_subscriber.caliberated_state
#     pose_subscriber.destroy_node()
#     return calib_state

def get_april_tag(rclpy, current_state):
    pose_subscriber = PoseSubscriber(current_state)
    rclpy.spin_once(pose_subscriber, timeout_sec=0.2)
    if pose_subscriber.calibrated_state:
        updated_state = pose_subscriber.calibrated_state
        #updated_state = current_state
    else:
        updated_state = current_state

    pose_subscriber.destroy_node()
    return updated_state

def coord(twist, current_state):
    J = np.array([[np.cos(current_state[2]), np.sin(current_state[2]), 0.0],
                  [-np.sin(current_state[2]), np.cos(current_state[2]), 0.0],
                  [0.0,0.0,1.0]])
    return np.dot(J, twist)

def zero_twist_msg():
    zero_twist = Twist()
    zero_twist.linear.x = 0.0
    zero_twist.linear.y = 0.0
    zero_twist.linear.z = 0.0
    zero_twist.angular.x = 0.0
    zero_twist.angular.y = 0.0
    zero_twist.angular.z = 0.0
    return zero_twist

if __name__ == "__main__":
    rclpy.init()

    import time
    #rospy.init_node("hw1")
    #pub_twist = rospy.Publisher("/twist", Twist, queue_size=1)
    """
    waypoint = np.array([[0.0,0.0,0.0], 
                         [-1.0,0.0,0.0],
                         [-1.0,1.0,np.pi/2.0],
                         [-2.0,1.0,0.0],
                         [-2.0,2.0,-np.pi/2.0],
                         [-1.0,1.0,-np.pi/4.0],
                         [0.0,0.0,0.0]]) 
    """
    #waypoint = np.array([[0.0, 0.0, 0.0],[1.0, 0.0, 0.0],[1.0, 2.0, np.pi/1.0],[0.0, 0.0, 0.0]])
    #waypoint = np.array([[0.0, 0.0, 0.0],[1,1,0.0],[2.2,1,0],[4,3.5,0.0]])
   #waypoint = np.array([[0.0, 0.0, 0.0],[0.75,0.75,0.0],[0.75,1.95,0],[1.5,3,0],[3,3,0.0]]) dijkstra
    #[[0.0, 0.0, 0.0], [0.1, 0.1, 0.0], [0.1, 0.2, 0.0], [0.2, 0.3, 0.0], [0.3, 0.4, 0.0], [0.4, 0.5, 0.0], [0.4, 0.6, 0.0], [0.4, 0.7, 0.0], [0.4, 0.8, 0.0], [0.4, 0.9, 0.0], [0.4, 1.0, 0.0], [0.4, 1.1, 0.0], [0.5, 1.2, 0.0], [0.5, 1.3, 0.0], [0.6, 1.4, 0.0], [0.7, 1.5, 0.0], [0.8, 1.6, 0.0], [0.9, 1.7, 0.0], [1.0, 1.8, 0.0], [1.1, 1.9, 0.0], [1.2, 1.9, 0.0], [1.3, 1.9, 0.0], [1.4, 1.9, 0.0], [1.5, 1.9, 0.0], [1.6, 1.9, 0.0], [1.7, 1.9, 0.0], [1.8, 1.9, 0.0], [1.9, 1.9, 0.0], [2.0, 2.0, 0.0]]
 
 #hw5
    #waypoint = np.array([[0, 0,0],[0,4.5,np.pi/2], [0, 9,np.pi/2],[4.5,9,np.pi], [9, 9,np.pi],[9,4.5,np.pi/2] ,[9, 0,0], [1, 0,0], [1, 8,0], [8, 8,0], [8, 1,0], [2, 1,0], [2, 7,0], [7, 7,0], [7, 2,0], [3, 2,0], [3, 6,0], [6, 6,0], [6, 3,0], [4, 3,0], [4, 5,0], [5, 5,0], [5, 4,0]])/3
   # waypoint = np.array([[0,0,0],[2.5,0,0],[2.8,1.5,np.pi],[0.35,1.5,np.pi],[0.35,0.5,0],[2.85,0.5,0],[3.15,2,np.pi],[0.65,2,np.pi],[0.65,1,0],[3.15,1,0],[3.45,2.5,np.pi],[0.95,2.5,np.pi],[0.95,1,0]])
   # waypoint = np.array([[0,0,0],[2.5,0,0],[2.8*1.2,1.7,np.pi],[0.35*1.2,1.7,np.pi],[0.35*1.2,0.7,0],[2.85*1.2,0.7,0],[3.15*1.2,2.5,np.pi],[0.65*1.2,2.5,np.pi],[0.65*1.2,1.5,0],[3.15*1.2,1.5,0],[3.45*1.2,3,np.pi],[0.95*1.2,3,np.pi],[0.95*1.2,2,0]])   
 #waypoint = np.array([[0,0,0],[2.5,0,0],[2.5,0.5,0],[0,0.5,0]])
   #waypoint = np.array([[0, 0,0], [0, 9,0], [1, 9,0], [1, 0,0], [2, 0,0], [2, 9,0], [3, 9,0], [3, 0,0], [4, 0,0], [4, 9,0], [5, 9,0], [5, 0,0], [6, 0,0], [6, 9,0], [7, 9,0], [7, 0,0], [8, 0,0], [8, 9,0], [9, 9,0], [9, 0,0]])/3
# init pid controller
    waypoint = np.array([[0,0,0],[1,0,0],[2,0,0],[3,0,0],[3,1.5,np.pi+0.25],[2,1.5,np.pi+0.25],[1,1.5,np.pi+0.25],[0,1.5,np.pi+0.25],[0,0.5,0],[1,0.3,0],[2,0.3,0],[3,0.3,0],[3,1.8,np.pi],[2,1.8,np.pi],[1,1.8,np.pi],[0,1.8,np.pi],[0,0.6,0-0.2],[1,0.6,0-0.2],[2,0.6,0-0.2],[3,0.6,0-0.2],[3,2.1,np.pi],[2,2.1,np.pi],[1,2.1,np.pi],[0,2.1,np.pi],[0,0.9,0],[1,0.9,0],[2,0.9,0],[3,0.9,0],[3,2.4,np.pi],[2,2.4,np.pi],[1,2.4,np.pi],[0,2.4,np.pi],[0,1.2,0],[1,1.2,0],[2,1.2,0],[3,1.2,0],[3,2.7,np.pi],[2,2.7,np.pi],[1,2.7,np.pi],[0,2.7,np.pi]])
    #waypoint = np.array([[0,0,0],[1,0,0],[2,0,0],[3,0,0],[3,1.2,np.pi],[2,1.2,np.pi],[1,1.2,np.pi],[0,1.2,np.pi],[0,0.6,0],[1,0.6,0],[2,0.6,0],[3,0.6,0],[3,1.8,np.pi],[2,1.8,np.pi],[1,1.8,np.pi],[0,1.8,np.pi],[0,1.2,0],[1,1.2,0],[2,1.2,0],[3,1.2,0],[3,2.4,np.pi+0.25],[2,2.4,np.pi],[1,2.4,np.pi],[0,2.4,np.pi] ,[0,1.8,0],[1,1.8,0],[2,1.8,0],[3,1.8,0],[3,3,np.pi],[2,3,np.pi],[1,3,np.pi],[0,3,np.pi]])#,[0,2.5,0],[1,2.5,0],[2,2.5,0],[3,2.5,0],[3,3.5,np.pi],[2,3.5,np.pi],[1,3.5,np.pi],[0,3.5,np.pi]])
    pid = PIDcontroller(0.02,0.005,0.005)

    # init current state
    current_state = np.array([0.0,0.0,0.0])

    # in this loop we will go through each way point.
    # once error between the current state and the current way point is small enough, 
    # the current way point will be updated with a new point.
    for wp in waypoint:
        print("move to way point", wp)
        # set wp as the target point
        pid.setTarget(wp)

        # calculate the current twist
        update_value = pid.update(current_state)
        # publish the twist
        pid.publisher_.publish(genTwistMsg(coord(update_value, current_state)))
        #print(coord(update_value, current_state))
        time.sleep(0.05)
        # update the current state
        current_state += update_value
        #current_state[2]=(current_state[2] +np.pi) % (2 * np.pi) - np.pi
        current_state[2]=round(current_state[2],2)
        start_t = time.time()
        while(np.linalg.norm(pid.getError(current_state, wp)) > 0.1): # check the error between current state and current way point
            # calculate the current twist
            update_value = pid.update(current_state)
            # publish the twist
            pid.publisher_.publish(genTwistMsg(coord(update_value, current_state)))
            #print(coord(update_value, current_state))
            time.sleep(0.05)
            # update the current state
            current_state += update_value
            #####camera feedbck
            curr_t = time.time()
            #print(curr_t-start_t)
            if (curr_t - start_t)>0.8 :
               # print(curr_t - start_t)
                 start_t = time.time()
                 pid.publisher_.publish(zero_twist_msg())
               #  time.sleep(1)
                 print(current_state)
                 calib_state = get_april_tag(rclpy,current_state)
                 #start_t = time.time()
                 current_state = calib_state


    # stop the car and exit
    pid.publisher_.publish(genTwistMsg(np.array([0.0,0.0,0.0])))

