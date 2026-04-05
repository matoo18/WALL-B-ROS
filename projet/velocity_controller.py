import rclpy
from rclpy.node import Node
import click

import threading
import copy
import numpy as np

#Messages needed
from nav_msgs.msg import Odometry #Odometry (Current Robot Position)
from sensor_msgs.msg import LaserScan #Lidar Messages
from geometry_msgs.msg import Twist, Point #Twist Message for velocity control

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
        self.visionSubscriber = self.create_subscription(Point, '/vision_target', self.vision_callback, 10)

        #Messages Init
        self.odomMsg = Odometry()
        self.sensorMsg = LaserScan()
        self.controlMsg = Twist()

        # Données de la vision
        self.vision_cible_x = 160.0
        self.vision_cam_centre = 160.0
        self.vision_is_tracking = False

        #Timer Initialisation 
        self.timer = self.create_timer(0.05, self.computeVelocity)

        #Controller Parameters
        self.ka = 0.2
        self.kr = 20
        self.maximum_repulsive_force_distance = 0.5

        self.k_angle = 0.5

    def odom_callback(self,msg):
        self.odomMsg = msg

    def scan_callback(self,msg):
        self.sensorMsg = msg

    def vision_callback(self, msg):
        self.vision_cible_x = msg.x
        self.vision_cam_centre = msg.y
        self.vision_is_tracking = bool(msg.z == 1.0)

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
    
    def angular_error(self, current_angle, desired_angle):
        #Determines the shortest path to take to achieve the desired orientation for the robot

        if (desired_angle > np.pi/2) and (desired_angle <= np.pi):
            if (current_angle > -np.pi) and (current_angle <= -np.pi/2):
                current_angle += 2*np.pi
        
        if (current_angle > np.pi/2) and (current_angle <= np.pi):
            if (desired_angle > -np.pi) and (desired_angle <= -np.pi/2):
                desired_angle += 2*np.pi

        error = desired_angle - current_angle
        return error

    def get_Robot_Pose(self):
        #Gives back the estimation of the robot position in the fixed frame and it's orientation [-pi,pi]
        robotPos = self.odomMsg.pose.pose.position
        robotOrient = self.odomMsg.pose.pose.orientation

        roll, pitch, yaw = self.quaternions_to_euler_angle(robotOrient) #Orientation in rad
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

        """
        for i in range(len(xo)):
            print("Object ", i+1, " coordinates: ", xo[i] - xRobot, yo[i] - yRobot)
        """

        return xo,yo
        
    def computeVelocity(self):
        rPos, rOrient = self.get_Robot_Pose()
        xObjects,yObjects = self.detect_Obstacles(rPos,rOrient)

        xRobot = rPos.x
        yRobot = rPos.y
        posRobot = np.array([xRobot,yRobot])

        # #Attractive Force Generation:
        # #To have an attractive force, there needs to be an objective for the robot to follow, for the time being, it's a point that always 0.25m infront of it.
        # point_distance = 0.25 #m
        # x_desired = xRobot + point_distance*np.cos(rOrient)
        # y_desired = yRobot + point_distance*np.sin(rOrient)
        # desired_Point = np.array([x_desired, y_desired])

        # attractionForce = - self.ka*(posRobot - desired_Point)

        # j'ai essayé de combiner nos 2 codes en modifiant uniquement ta variable d'attraction pour essayer de la modifiée dynamiquement avec mon topic
        # de suivi de ligne mais ça marche pas pour l'instant mdr.
        # On calcule l'angle désiré depuis l'erreur visuelle
        error_x = self.vision_cam_centre - self.vision_cible_x
        vision_angular = float(error_x * 0.005)
        
        # On place notre aimant d'attraction dans la direction de la courbe
        point_distance = 0.25 
        desired_angle = rOrient + vision_angular
        desired_Point = np.array([rPos.x + point_distance * np.cos(desired_angle),
                                    rPos.y + point_distance * np.sin(desired_angle)])

        attractionForce = - self.ka * (posRobot - desired_Point)
        
        #Repulsive force generation
        repulsiveForce = np.zeros(2,)
        for i in range(len(xObjects)):
            objPos = np.array([xObjects[i],yObjects[i]])
            distance = np.linalg.norm(posRobot - objPos)
            if(distance <= self.maximum_repulsive_force_distance):
                forceNorm = self.kr*((1/self.maximum_repulsive_force_distance) -(1/distance))*(1/(distance)**2)
                repulsiveForce -= forceNorm*(posRobot - objPos)

        controlForce = attractionForce + repulsiveForce

        goal_Orientation = np.arctan2(controlForce[1],controlForce[0])
        
        angleError = self.angular_error(rOrient,goal_Orientation)

        angular_velocity = self.k_angle*angleError
        linear_velocity = np.linalg.norm(controlForce)
        if(np.abs(linear_velocity) > 2.0):
            linear_velocity = 2.0
        
        #print(linear_velocity)
        #print(angular_velocity)

        self.controlMsg.linear.x = linear_velocity
        self.controlMsg.linear.y = 0.0
        self.controlMsg.linear.z = 0.0
        self.controlMsg.angular.x = 0.0
        self.controlMsg.angular.y = 0.0
        self.controlMsg.angular.z = angular_velocity
        
        self.controlPublisher.publish(self.controlMsg)


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

