import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
import numpy as np


class PoseSubscriber(Node):
    def init(self, current_state):
        super().init('pose_subscriber')
        self.calibrated_state = []  # List to store the updated robot state
        self.subscription = self.create_subscription(
            PoseArray,
            'april_poses',  # Replace with your actual topic name
            self.pose_callback,
            10)
        self.ld = {  # Map of tag IDs to handling rules
            "marker_6": [4,0],
            "marker_3": [4,-0.7],
            "marker_17": [1,1.2],
            "marker_1": [3.5,4],
            "marker_4": [4,3.5],
            "marker_15":[1.2,1],
            "marker_8":[1.2,1.2]
        }
        self.cs_x = current_state[0]  # Current x state
        self.cs_y = current_state[1]  # Current y state
        self.cs_w = current_state[2]  # Current orientation
        print("Initial current state:", current_state)

    def pose_callback(self, msg):
        if not msg.poses:
            print("No tags detected.")
            return
        print('hi')
        pose_ids = msg.header.frame_id.split(',')[:-1]  # Assuming frame_id stores tag IDs
        for i, pose in enumerate(msg.poses):
            tag_id = pose_ids[i] if i < len(pose_ids) else f"Tag_{i}"
            x, y, z = pose.position.x, pose.position.y, pose.position.z
            qx, qy, qz, qw = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w

            theta = quaternion_to_pitch(qx, qy, qz, qw)

            print(f"Detected tag ID: {tag_id}, Pose: x={z}, y={x}, theta={theta}")
         
            ld_wx,ld_wy = self.ld[tag_id]
            ld_x, ld_y = z,x
            
         
            x_w, y_w = transform_to_world_frame(ld_x,ld_y,ld_wx,ld_wy, theta)
            print("robot world: ",x_w,y_w)
            w = 0.99
            self.calibrated_state = [0.8*self.cs_x+(1-0.2)*x_w, w*self.cs_y+(1-w)*y_w, self.cs_w]
            print("Updated calibrated state:", self.calibrated_state)
import math

def transform_to_world_frame(x_r, y_r, x_l, y_l, theta):
    # Compute robot coordinates in the world frame
    x_w = x_l - x_r * math.cos(theta) + y_r * math.sin(theta)
    y_w = y_l - x_r * math.sin(theta) - y_r * math.cos(theta)
    return x_w, y_w

# Example: Using pitch as theta


# Compute pitch (theta)
def quaternion_to_pitch(qx, qy, qz, qw):
    sin_pitch = 2 * (qw * qy - qz * qx)
    sin_pitch = max(-1.0, min(1.0, sin_pitch))  # Clamp for numerical stability
    return math.asin(sin_pitch)



def main(args=None):
    rclpy.init(args=args)
    initial_state = [0.0, 0.0, 0.0]  # Initial state for the robot
    pose_subscriber = PoseSubscriber(initial_state)

    try:
        rclpy.spin(pose_subscriber)
    except KeyboardInterrupt:
        print("Shutting down node.")

    pose_subscriber.destroy_node()
    rclpy.shutdown()


if name == 'main':
    main()
