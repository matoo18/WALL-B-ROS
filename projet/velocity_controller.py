import rclpy
from rclpy.node import Node
import click

import threading
import copy
import numpy as np

#Messages needed
from nav_msgs.msg import Odometry #Odometry (Current Robot Position)
from sensor_msgs.msg import LaserScan #Lidar Messages
from geometry_msgs.msg import Twist #Twist Message for velocity control

#from std_msgs.msg import Float32MultiArray

import math
import time


class ControllerNode(Node):

    def __init__(self):
        super().__init__('lds_distance')
        
        #Publishers and Subscribers Init
        self.controlPublisher = self.create_publisher(Twist, '/cmd_vel', 10) #Command Publisher
        self.odometrySubscriber = self.create_subscription(Odometry, '/odom', self.odom_callback,10)
        self.sensorSubscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        #Messages Init
        self.odomMsg = Odometry()
        self.sensorMsg = LaserScan()
        self.controlMsg = Twist()

        #self.message = copy.deepcopy(Float32MultiArray())
        #Timer Initialisation 
        self.timer = self.create_timer(0.05, self.computeVelocity)

    def odom_callback(self,msg):
        self.odomMsg = msg

    def scan_callback(self,msg):
        self.sensorMsg = msg

    def quaternions_to_euler_angle(self, quaternion):
        #Converts a given quaternion to euler angles
        qx = quaternion.x
        qy = quaternion.y
        qz = quaternion.z
        qw = quaternion.w

        roll = np.arctan2(qw*qx + qy*qz, 1 - 2*(qx**2 + qy**2))
        pitch = -np.pi/2 + 2*np.arctan2(np.sqrt(1 + 2*(qw*qy - qx*qz)), np.sqrt(1 - 2*(qw*qy - qx*qz)))
        yaw = np.arctan2(2*(qw*qz + qx*qy), 1-2*(qy**2 + qz**2))

        return roll,pitch,yaw
    
    def get_Robot_Pose(self):
        #Gives back the estimation of the robot position in the fixed frame and it's orientation [-pi,pi]
        robotPos = self.odomMsg.pose.pose.position
        robotOrient = self.odomMsg.pose.pose.orientation

        roll, pitch, yaw = self.quaternions_to_euler_angle(robotOrient) #Orientation in rad/s
        return robotPos, yaw

    
    def detect_Obstacles(self, robotPos, robotOrientation):
        #Discriminates the obstacles in range of the LIDAR, it gives the smallest distance detected by a ray and the angle alpha of the ray in reference to the robot.
        start_angle = self.sensorMsg.angle_min
        increment_angle = self.sensorMsg.angle_increment
        lidarRanges = np.array(self.sensorMsg.ranges)

        xRobot = robotPos.x
        yRobot = robotPos.y

        #Gives an array of indices where the ranges are not == inf
        not_inf_ranges_indices = np.where(~np.isinf(lidarRanges))[0] #[0] to retrieve the non numpy array.

        #Calculates the differences between neighboring values if there's a "jump" then it can considered that's where an object ends.

        differences = np.diff(not_inf_ranges_indices)

        index_for_split = np.where(np.abs(differences) > 1)[0] + 1 #Indices where the values "jump", we add 1 as to signify the end of the obstacle

        if len(not_inf_ranges_indices) == 0: #If no obstacles, return empty arrays
            return [], []

        split_arrays = np.split(not_inf_ranges_indices, index_for_split) #Partioned arrays where each array represents an obstacle and the distances measured by each ray from the robot

        
        min_objects_distances = [] #List of minimum distance for each object
        min_objects_distances_angles = [] #List of the ray angle for the minimal distance
        
        for i in range(len(split_arrays)):
            obstacle_distances = lidarRanges[split_arrays[i]]
            min_index = np.argmin(obstacle_distances)
            min_objects_distances.append(min(obstacle_distances))
            min_objects_distances_angles.append(start_angle +increment_angle*split_arrays[i][min_index])


        #Pair of the smallest distance from the object x and y coordinates 
        xo = []
        yo = []

        for i in range(len(min_objects_distances)):
            xo.append(xRobot + min_objects_distances[i]*np.cos(min_objects_distances_angles[i]+robotOrientation))
            yo.append(yRobot + min_objects_distances[i]*np.sin(min_objects_distances_angles[i]+robotOrientation))

        for i in range(len(xo)):
            print("Object ", i+1, " coordinates: ", xo[i], yo[i])
        return xo,yo
        

    def computeVelocity(self):
        rPos, rOrient = self.get_Robot_Pose()
        self.detect_Obstacles(rPos,rOrient)

def main(args=None):
    try:
        rclpy.init()
        node = ControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()

