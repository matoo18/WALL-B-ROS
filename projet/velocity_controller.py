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
from std_msgs.msg import Int32
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
        self.challengeSubscriber = self.create_subscription(Int32, '/active_state', self.state_callback, 10)

        #Messages Init
        self.etat_actuel = 1
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

    def state_callback(self, msg):
        self.etat_actuel = msg.data

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
        if self.etat_actuel != 2:
            return

        # ==========================================================
        # LOGIQUE DE VISION 
        # ==========================================================
        error_x = self.vision_cam_centre - self.vision_cible_x
        vision_angular = float(error_x * 0.005)

        # ==========================================================
        # LOGIQUE D'ESQUIVE 
        # ==========================================================
        rPos, rOrient = self.get_Robot_Pose()
        xObjects, yObjects = self.detect_Obstacles(rPos, rOrient)
        
        avoidance_angular = 0.0
        
        for i in range(len(xObjects)):
            # distance entre le robot et chacun des obstacles grace pytha
            dx = xObjects[i] - rPos.x
            dy = yObjects[i] - rPos.y
            distance = math.hypot(dx, dy)
            
            # obstacle dans la zone de sécurité ?
            if distance <= self.maximum_repulsive_force_distance:
                
                # TOA : calcul de l'angle entre l'hypothénuse et la ligne horizontale imaginaire entre la position x de l'objet et la projection d ela posx du robot
                # arrctan2 retourne un angle entre -pi et pi
                global_angle = np.arctan2(dy, dx)
                # on a calculé l'angle entre l'horizontale et l'obstacle et on a aussi l'orientation du robot selon l'horizontale
                # du coup, on soustrayant les 2 on trouve l'angle entre l'obstacle et l'orientation du robot
                theta = global_angle - rOrient
                
                # angle entre -pi et pi 
                theta = (theta + np.pi) % (2 * np.pi) - np.pi
                
                # si l'obstacle est à plus de 90° alors ça veut dire qu'il derrière à gauche du robot donc on s'en occupe pas ou plus
                # pareil pour -90° ça veut dire qu'il est derrière à droite donc on s'en occupe pas ou plus non plus 
                if abs(theta) < (np.pi / 2):
                    
                    # plus on est près, plus on veut réagir fortement donc on créé un pods
                    poids_dist = (self.maximum_repulsive_force_distance - distance) / self.maximum_repulsive_force_distance
                    
                    # ensuite en fonction de la projection de l'angle on va aussi vouloir pousser plus ou moins fort
                    impact_angle = math.cos(theta)
                    
                    # Si theta > 0 = obstacle à gauche, on tourne à droite -1
                    # Si theta < 0 = obstacle à droite, on tourne à gauche +1
                    direction = -1.0 if theta > 0 else 1.0
                    
                    # si pile en face je mets par défaut qu'on tourne à gauche
                    # (c'est un peu de la tricche parce que je sais que dans le parcours c'est le cas qui peut potentiellement arriver.)
                    if abs(theta) < 0.05:
                        direction = 1.0 
                        
                    # Calcul de la commande de répulsion
                    repulsion = direction * poids_dist * impact_angle
                    
                    # si y'a plusieurs obstacles on garde celui dont la répulsion est la plus forte. On pourrait aussi le modifier avec des sortes de poids mais ça fait le taf.
                    if abs(repulsion) > abs(avoidance_angular):
                        avoidance_angular = repulsion

        # ==========================================================
        # COMBINAISON RÉPULSION ET VISION
        # ==========================================================
        # gain évitement (empirique)
        K_avoid = 2
        
        final_angular_velocity = vision_angular + (K_avoid * avoidance_angular)

        # je réutilise ma logique de proportion que j'ai utilisé pour l'épreuve 1
        linear_velocity = 0.22 - 0.4 * abs(final_angular_velocity)
        linear_velocity = max(0.05, min(0.22, linear_velocity))

        self.controlMsg.linear.x = float(linear_velocity)
        self.controlMsg.linear.y = 0.0
        self.controlMsg.linear.z = 0.0
        self.controlMsg.angular.x = 0.0
        self.controlMsg.angular.y = 0.0
        self.controlMsg.angular.z = float(final_angular_velocity)
        
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

