import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray,PoseStamped, Twist
from nav_msgs.msg import Odometry
import numpy as np
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseArray
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

trajectory = []
landmark_positions = []
landmark_covariances = []
landmark_id= []
landmark_positions_new = []
current_state_arr = []

class PoseSubscriber(Node):
    def __init__(self):
        super().__init__('pose_subscriber')
        self.calibrated_state = []
        self.subscription = self.create_subscription(
            PoseArray,
            'april_poses',  # Topic name must match the C++ publisher
            self.pose_callback,
            10)

    def pose_callback(self, msg):
        global april_poses,state, covariance,landmarks,Q,R,pose_ids,flag
        pose_ids = msg.header.frame_id.split(',')[:-1]
        april_poses = msg.poses 
        flag =1 


 
def twist_callback(msg):
    #    v_x = msg.twist.twist.linear.x
    #    v_y = msg.twist.twist.linear.y
    #    omega = msg.twist.twist.angular.z
        global april_poses,state, covariance,landmarks,Q,R
        v_x = msg.linear.x
        v_y = msg.linear.y
        omega = msg.angular.z
        dt = 0.8
        print("velocities and time in odom : ",v_x, v_y, omega, dt)
     
      #  predict(24*v_x,20*v_y,28*omega,dt)
        predict(25*v_x,22*v_y,28*omega,dt)
def apriltag_callback_array():
        # Extract AprilTag measurements (assumed to be in the camera frame)
        global april_poses,state, covariance,landmarks,Q,R,pose_ids
     #   print("hi_array")
        for i, pose in enumerate(april_poses):
            print("detected tag = ",pose_ids[i])
            # print(f"Pose {i}: x={pose.position.x}, y={pose.position.y}, z={pose.position.z}")

            lx_cam = pose.position.z
            ly_cam = pose.position.x
           # print("values from april tag ",lx_cam," ",ly_cam)
            # Get the robot's current pose in the world frame
            x_robot =  state[0, 0]
            y_robot =  state[1, 0]
            theta_robot =  state[2, 0]
            
            # Transformation matrix from the robot frame to the world frame
            transformation_matrix = np.array([
   [np.cos(theta_robot), -np.sin(theta_robot), x_robot],
   [np.sin(theta_robot), np.cos(theta_robot), y_robot],
   [0, 0, 1]
            ])

            # Landmark position in the camera frame (homogeneous coordinates)
            landmark_cam = np.array([[lx_cam], [ly_cam], [1]])

            # Convert the landmark position to the world frame
            landmark_world = np.dot(transformation_matrix, landmark_cam)
            print("landmark world: ",landmark_world)
            # Extract world coordinates of the landmark
            lx_world = landmark_world[0, 0]
            ly_world = landmark_world[1, 0]
            landmark_id.append([pose_ids[i],lx_world,ly_world])
            # Update the EKF state with the converted landmark position
            update(np.array([[lx_world], [ly_world]]))
def apriltag_callback2():
            global april_poses,state, covariance,landmarks,Q,R
            pose = april_poses
            lx_cam = pose.position.z
            ly_cam = pose.position.x
          #  print("values from april tag ",lx_cam," ",ly_cam)
            # Get the robot's current pose in the world frame
            x_robot =  state[0, 0]
            y_robot =  state[1, 0]
            theta_robot =  state[2, 0]

            # Transformation matrix from the robot frame to the world frame
            transformation_matrix = np.array([
   [np.cos(theta_robot), -np.sin(theta_robot), x_robot],
   [np.sin(theta_robot), np.cos(theta_robot), y_robot],
   [0, 0, 1]
            ])

            # Landmark position in the camera frame (homogeneous coordinates)
            landmark_cam = np.array([[lx_cam], [ly_cam], [1]])

            # Convert the landmark position to the world frame
            landmark_world = np.dot(transformation_matrix, landmark_cam)
           # print("landmark world: ",landmark_world)
            # Extract world coordinates of the landmark
            lx_world = landmark_world[0, 0]
            ly_world = landmark_world[1, 0]

            # Update the EKF state with the converted landmark position
          #  update(np.array([[lx_world], [ly_world]]))
def predict(v_x, v_y, omega, dt):
        """Predicts the next state using the control inputs."""
        global april_poses,state, covariance,landmarks,Q,R
       # print("hi")
       # theta =  normalize_angle(state[2, 0])
        theta = state[2,0]
        # Predict the change in x and y considering vx and vy
        dx = (v_x * math.cos(theta) - v_y * math.sin(theta)) * dt
        dy = (v_x * math.sin(theta) + v_y * math.cos(theta)) * dt
        dtheta = omega * dt

        # Update the robot's state
        state[0, 0] += dx
        state[1, 0] += dy
        state[2, 0] += dtheta
        state[2,0] = normalize_angle(state[2,0])
        print("predict step  state : ", [state[0,0],state[1,0],state[2,0]])
        trajectory.append([state[0, 0], state[1, 0]])
        # Compute the Jacobian of the motion model
        G = np.eye(state.shape[0])
        G[0, 2] = (-v_x * math.sin(theta) - v_y * math.cos(theta)) * dt
        G[1, 2] = (v_x * math.cos(theta) - v_y * math.sin(theta)) * dt
        # Ensure Q is the correct size (expand it if necessary)
        n = state.shape[0]
        if Q.shape[0] != n:
            new_Q = np.zeros((n, n))
            new_Q[:3, :3] = Q[:3, :3]  # Copy the original Q for robot state
            Q = new_Q
      #  print("covariance in predict step = ",covariance)
        # print("Q =",Q)
        # Update the covariance
        covariance = G @  covariance @ G.T +  Q
        #publish_estimated_pose()

def update(measurement):
        """Updates the state based on landmark observations."""
        global april_poses,state, covariance,landmarks,Q,R
        lx, ly = measurement.flatten()

        # Check if the landmark is already known
        if not is_landmark_known(lx, ly):
            # Add new landmark to the state vector
            state = np.vstack([ state, [[lx], [ly]]])

            # Expand the covariance matrix
         
         #   new_covariance = np.eye(len( state)) * 1000
            new_covariance = np.eye(len(state)) * 1000
            new_covariance[:len( covariance), :len( covariance)] =  covariance
            covariance = new_covariance

        # Get the index of the landmark in the state vector
        idx =  get_landmark_index(lx, ly)

        # Extract current landmark position from the state vector
        lx, ly =  state[idx:idx+2, 0]

        # Compute expected measurement
        dx = lx -  state[0, 0]
        dy = ly -  state[1, 0]
        q = dx**2 + dy**2
        z_pred = np.array([[math.sqrt(q)], [math.atan2(dy, dx) -  state[2, 0]]])

        # Compute the Jacobian of the measurement model
        H = np.zeros((2, len( state)))
        H[0, 0] = -dx / math.sqrt(q)
        H[0, 1] = -dy / math.sqrt(q)
        H[1, 0] = dy / q
        H[1, 1] = -dx / q
        H[0, idx] = dx / math.sqrt(q)
        H[1, idx] = -dy / q

        # Measurement update
        S = H @  covariance @ H.T +  R
        K =  covariance @ H.T @ np.linalg.inv(S)
        y = measurement - z_pred
        y[1, 0] =  normalize_angle(y[1, 0])  # Normalize angle difference
        state += K @ y
        covariance = (np.eye(len( state)) - K @ H) @  covariance
        
        for i in range(3, len(state), 2):
            landmark_positions.append([state[i, 0], state[i + 1, 0]])
            landmark_covariances.append(covariance[i:i + 2, i:i + 2])
        # Publish the updated pose
      #  publish_estimated_pose()

def is_landmark_known(lx, ly, threshold=1.6):
        """Check if a landmark is already known."""
        global april_poses,state, covariance,landmarks,Q,R
        for i in range(3, len( state), 2):
            known_lx, known_ly =  state[i:i+2, 0]
            if np.hypot(lx - known_lx, ly - known_ly) < threshold:
                         return True
     
        return False

def get_landmark_index(lx, ly, threshold=1.6):
        """Get the index of a known landmark."""
        global april_poses,state, covariance,landmarks,Q,R
        for i in range(3, len( state), 2):
            known_lx, known_ly =  state[i:i+2, 0]
            if np.hypot(lx - known_lx, ly - known_ly) < threshold:
                            return i
        return None

def publish_estimated_pose():
        """Publish the current estimated pose of the robot."""
        global april_poses,state, covariance,landmarks,Q,R,current_state_arr
        pose_msg = PoseStamped()
        #pose_msg.header.stamp =  get_clock().now().to_msg()
        pose_msg.pose.position.x =  state[0, 0]
        pose_msg.pose.position.y =  state[1, 0]
        pose_msg.pose.orientation.z = math.sin( state[2, 0] / 2)
        pose_msg.pose.orientation.w = math.cos( state[2, 0] / 2)
        print("published self state = ", state)
        global current_state
        current_state_arr.append([current_state[0],current_state[1],current_state[2]])

        w = 0.7
        # or w as 0.7 for best
        current_state = [((1-0.5)*state[0,0]+0.5*current_state[0]),((1-w)*state[1,0]+w*current_state[1]),((1-w)*state[2,0]+w*current_state[2])]
        print("current state after update =",current_state)
       #S print("published state= ",[state[0,0],state[1,0],state[2,0]])
        # odom_publisher.publish(pose_msg)


def normalize_angle(angle):
        """Normalize an angle to be within [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    

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
        self.publisher_2 = self.create_publisher(Twist, '/twist2', 10)
       
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

def get_april_tag(rclpy):
    global flag
    flag =0
    pose_subscriber = PoseSubscriber()
    rclpy.spin_once(pose_subscriber)
    print("hi")
    if flag: #handling case when it doesnt see apri tag
        apriltag_callback_array()
    pose_subscriber.destroy_node()



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

def plot_covariance_ellipse(position, covariance, n_std=2.0, **kwargs):
    """Create a plot of the covariance ellipse."""
    eigenvalues, eigenvectors = np.linalg.eig(covariance)
    order = eigenvalues.argsort()[::-1]
    eigenvalues, eigenvectors = eigenvalues[order], eigenvectors[:, order]

    angle = np.arctan2(eigenvectors[1, 0], eigenvectors[0, 0])
    angle = np.degrees(angle)
    width, height = 2 * n_std * np.sqrt(eigenvalues)

    return Ellipse(
        xy=position, width=width, height=height, angle=angle,
        edgecolor='green', facecolor='none', linewidth=1, **kwargs
    )

def plot_trajectory_and_landmarks2(trajectory, landmark_positions, landmark_covariances):
    fig, ax = plt.subplots()
    global landmark_id,covariance,state
    # Plot the trajectory
    trajectory = np.array(trajectory)
    ax.plot(trajectory[:, 0], trajectory[:, 1], label='Estimated Trajectory', color='blue')
    s = state[3:]
    # Plot the landmarks and their covariances
    # for pos, cov in zip(landmark_positions, landmark_covariances):
    #     ax.scatter(pos[0], pos[1], color='red')
        # ellipse = plot_covariance_ellipse(pos, cov)
        # ax.add_patch(ellipse)
    for i in range(0,len(s),2):
         ax.scatter(s[i],s[i+1],color='red')

    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.legend()
    ax.set_title('Estimated Trajectory and Landmarks')
    plt.grid()
    plt.show()
    plt.savefig('path1.png')
    global state
    with open('state_landmark_txt','w') as f:
         f.write(str(state))
         f.write("\nlandmakr covariance\n")
         f.write(str(covariance))
         f.write("\ntrajectory\n")
         f.write(str(trajectory))
         f.write("\n landmark pos with id\n")
         f.write(str(landmark_id))
         f.write("\n landmark positions new\n")
         f.write(str(landmark_positions_new))
         f.write("\n arrays of current state\n")
         f.write(str(current_state_arr))

def plot_landmakrs_unique():
    global landmark_positions_new,landmark_id
    marker_dict = {}

    # Group data by marker
    for marker, x, y in landmark_id:
        if marker not in marker_dict:
            marker_dict[marker] = {'x': [], 'y': []}
        marker_dict[marker]['x'].append(x)
        marker_dict[marker]['y'].append(y)

    # Calculate mean x, y for each marker and append to an array
    mean_values = []
    for marker, data in marker_dict.items():
        mean_x = np.mean(data['x'])
        mean_y = np.mean(data['y'])
        landmark_positions_new.append([marker, mean_x, mean_y])


def plot_trajectory_and_landmarks(trajectory, landmark_positions, landmark_covariances):
    global landmark_positions_new,state,covariance
    plot_landmakrs_unique()

    fig, ax = plt.subplots()

    # Plot the trajectory
    trajectory = np.array(trajectory)
    ax.plot(trajectory[:, 0], trajectory[:, 1], label='Estimated Trajectory', color='blue')
    s = state[3:]
    c = covariance[3:]
    # landmark_covariances = [0.05 *i for i in landmark_covariances]
    # Plot the landmarks and their covariance as crosses
    # for pos, cov in zip(landmark_positions, landmark_covariances):
    #     ax.scatter(pos[0], pos[1], color='red')

    #     # Calculate the eigenvalues and eigenvectors for covariance direction
    #     eigenvalues, eigenvectors = np.linalg.eig(cov)
    #     order = eigenvalues.argsort()[::-1]
    #     eigenvalues, eigenvectors = eigenvalues[order], eigenvectors[:, order]

    #     # Plot crosses for the principal directions of the covariance
    #     for i in range(2):  # Two principal directions
    #         vector = eigenvectors[:, i] * np.sqrt(eigenvalues[i])
    #         ax.plot([pos[0] - vector[0], pos[0] + vector[0]], [pos[1] - vector[1], pos[1] + vector[1]], color='green')

    for j in range(0, len(s), 2):  # Use 'j' to iterate over positions in 's'
        ax.scatter(s[j], s[j+1], color='red')

        # Calculate the eigenvalues and eigenvectors for covariance direction
        eigenvalues, eigenvectors = np.linalg.eig(c[j:j+2, j:j+2])
        order = eigenvalues.argsort()[::-1]
        eigenvalues, eigenvectors = eigenvalues[order], eigenvectors[:, order]

        # Ensure eigenvalues are non-negative before taking the square root
        eigenvalues = [abs(i) for i in eigenvalues]
        eigenvalues = np.clip(eigenvalues, 0, None)

        # Plot crosses for the principal directions of the covariance
        for k in range(2):  # Use 'k' to avoid reusing the loop variable
            vector = eigenvectors[:, k] * np.sqrt(eigenvalues[k])
            ax.plot([s[j] - vector[0], s[j] + vector[0]], [s[j+1] - vector[1], s[j+1] + vector[1]], color='green')
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.legend()
    ax.set_title('Estimated Trajectory and Landmarks with Covariance Crosses')
    plt.grid()
    plt.show()
    plt.savefig('path2.png')


if __name__ == "__main__":
    rclpy.init()
    global Twist1
    import time
    #rospy.init_node("hw1")
    #pub_twist = rospy.Publisher("/twist", Twist, queue_size=1)
    global april_poses,state, covariance,landmarks,Q,R
   # covariance = np.eye(3) * 0.1  # Initial covariance
    covariance = np.eye(3)*0.1
    landmarks = {}  # Dictionary to store landmarks
    Q = np.eye(3) * 0.01  # Process nois
    #R = np.eye(2) * 0.1  # Measurement noise
  #  Q = np.array([[0.1, 0, 0],[0, 0.25, 0],[0, 0, np.deg2rad(20)**2]])
  #  R = np.array([[0.01, 0],[0, 0.1]])
    R = np.array([[1.3*0.05, 0],[0, 1.3*0.09]])
    state = np.zeros((3, 1))  # [x, y, theta] - Robot state in world fr
    """
    waypoint = np.array([[0.0,0.0,0.0], 
                         [-1.0,0.0,0.0],
                         [-1.0,1.0,np.pi/2.0],
                         [-2.0,1.0,0.0],
                         [-2.0,2.0,-np.pi/2.0],
                         [-1.0,1.0,-np.pi/4.0],
                         [0.0,0.0,0.0]]) 
    """
    # waypoint = np.array([[0.0, 0.0, 0.0],[2, 0.0, 0.0],[2,0.5,0.2+np.pi/2.0],[2, 2,0.2+np.pi/2.0],[1.5,2,0.2+np.pi/1.0],[0.5,2,0.2+np.pi/1.0],[0.0,1.5,0.2-np.pi/2.0],[0,0.0,0.2-np.pi/2.0]])
    waypoint = np.array([[ 1.462,  0.   ,  0.     ],   # Point 1
    [ 1.035,  1.035,  0.7854 ],   # Point 2
    [ 0.   ,  1.462,  1.5708 ],   # Point 3
    [-1.035,  1.035,  2.3562 ],   # Point 4
    [-1.462,  0.   ,  3.1416 ],   # Point 5
    [-1.035, -1.035, -2.3562 ],   # Point 6
    [ 0.   , -1.462, -1.5708 ],   # Point 7
    #  [ 1.035, -1.035, -0.7854 ],
    #  [0.0,0.0,0.0],
    #  [ 1.462,  0.   ,  0.     ],   # Point 1
    #  [ 1.035,  1.035,  0.7854 ],   # Point 2
    #  [ 0.   ,  1.462,  1.5708 ],   # Point 3
    #  [-1.035,  1.035,  2.3562 ],   # Point 4
    #  [-1.462,  0.   ,  3.1416 ],   # Point 5
    #  [-1.035, -1.035, -2.3562 ],   # Point 6
    #  [ 0.   , -1.462, -1.5708 ],   # Point 7
    [ 1.035, -1.035, -0.7854 ]]   # Point 8
    )   # Point 8

    # init pid controller
    pid = PIDcontroller(0.02,0.005,0.005)

    # init current sta5
    global current_state
    current_state = np.array([0.0,0.0,0.0])
   
    # in this loop we will go through each way point.
    # once error between the current state and the current way point is small enough, 
    # the current way point will be updated with a new point.
    t1 = time.time()
    thresh = 0.2
    for wp in waypoint:
        print("move to way point", wp)
        # set wp as the target point
        pid.setTarget(wp)

        # calculate the current twist
        update_value = pid.update(current_state)
        # publish the twist
        pid.publisher_.publish(genTwistMsg(coord(update_value, current_state)))
        #print(coord(update_value, current_state))
        t2=time.time()
        time.sleep(0.05)
        # update the current state
        current_state += update_value
        start_t = time.time()
        thresh +=0.1
        while(np.linalg.norm(pid.getError(current_state, wp)) > thresh): # check the error between current state and current way point
            # calculate the current twist
            update_value = pid.update(current_state)
            # publish the twist
            pid.publisher_.publish(genTwistMsg(coord(update_value, current_state)))
            pid.publisher_2.publish(genTwistMsg(coord(update_value, current_state)))
            #print(coord(update_value, current_state))
            time.sleep(0.05)
            # update the current state
            current_state += update_value
            current_state[2] = normalize_angle(current_state[2])
            #####kalman feedbck
            curr_t = time.time()
            Twist1 = genTwistMsg(coord(update_value, current_state))
             
            if (curr_t - start_t)>0.8 :
               # print(curr_t - start_t)
                start_t = time.time()

                # pid.publisher_.publish(zero_twist_msg())
                # time.sleep(0.2)
                print("current state = ",current_state)
               #  caliberated_state = get_april_tag(rclpy,current_state)
                twist_callback(Twist1)
                get_april_tag(rclpy)
                publish_estimated_pose()
                start_t = time.time()
                # current_state = calib_state
        if((t2-t1)>9):
            break


   
    # stop the car and exit
    pid.publisher_.publish(genTwistMsg(np.array([0.0,0.0,0.0])))

    plot_trajectory_and_landmarks(trajectory, landmark_positions, landmark_covariances)    
    plot_trajectory_and_landmarks2(trajectory, landmark_positions, landmark_covariances)    

