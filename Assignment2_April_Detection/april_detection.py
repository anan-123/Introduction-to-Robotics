import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math
import numpy as np
class PoseSubscriber(Node):
    def __init__(self,current_state):
        super().__init__('pose_subscriber')
        self.calibrated_state = []
        self.subscription = self.create_subscription(
            PoseStamped,
            'april_poses',  # Topic name must match the C++ publisher
            self.pose_callback,
            10)
        self.ld = {
            "0": ['x', [1.4,0]],
            "1": ['y', [0, 0]],
            "2": ['vx', [-0.9,2]],
            "3": ['wp',[-0.9,0.6]],
            "4": ['f',[0,0]]
        }
        self.cs_x = current_state[0]
        self.cs_y = current_state[1]
        self.cs_w = current_state[2]
        print("current state  = ",current_state)

    def pose_callback(self, msg):
        # Print the detected pose
     #    print(msg.header.stamp)
         print('tag_id = ',msg.header.frame_id)
         landmark_id = msg.header.frame_id
     #    x = msg.pose.position.z
      #   y = msg.pose.position.x

         x = msg.pose.position.x
         y = msg.pose.position.y
         z = msg.pose.position.z
         print("april tag vector = ",[x,y,z])
         direction,p = self.ld[landmark_id]
         ld_x,ld_y = p
 #        if direction == 'x':
  #           self.cs_x = ld_x - z
         if direction == 'wp':
             self.cs_x = ld_x+z
         if direction == 'f':
             self.cs_x = x
            # self.cs_w = 3.8
          #    self.cs_y= ld_y

      #   if direction == 'vx':
        #      self.cs_x = 1
         #     self.cs_y = 2
             # self.cs_w = np.pi
       #  if direction == 'y':
        #       self.cs_y = ld_y + z 
            # self.cs_x = ld_x-x
         self.caliberated_state = [self.cs_x,self.cs_y,self.cs_w]
         print("caliberated iicurent state  = ",self.caliberated_state)
    # Prinuse the x, y, and yaw values
        # print(
         #   f"AprilTag Position in Camera Frame:\n"
          #  f"x: {x}, y: {y}\n"
           # f"Rotation (yaw) in camera frame: {yaw} radians"
        #)
       # print(
        #   f"Detected AprilTag Pose:\n"
         #   f"Position -> x: {msg.pose.position.x}, y: {msg.pose.position.y}, z: {msg.pose.position.z}\n"
         #   f"Orientation -> x: {msg.pose.orientation.x}, y: {msg.pose.orientation.y}, "
         #   f"z: {msg.pose.orientation.z}, w: {msg.pose.orientation.w}"
       # )


def main(args=None):
    rclpy.init(args=args)
    pose_subscriber = PoseSubscriber()
    rclpy.spin(pose_subscriber)

    # Cleanup
    pose_subscriber.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()

