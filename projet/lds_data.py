import rclpy
from rclpy.node import Node
import click
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import threading
import copy

import numpy as np

MEAN_RANGE = 20 #+- mean_range degrees near from main direction axis
NBR_SENSORS = 8 #First Sensor is at the "Back" of the robot and the next sensors follow the trigonometric direction

class ldsDistanceNode(Node):
    def __init__(self):
        super().__init__('lds_distance')

        self.publisher_ = self.create_publisher(Float32MultiArray, '/lds_means', 10)
        self.subscription_ = self.create_subscription(LaserScan, '/scan', self.scan_callback,10)
        self.message = copy.deepcopy(Float32MultiArray()) 

    def scan_callback(self, msg):
        min_angle = msg.angle_min
        max_angle = msg.angle_max
        
        range_min = msg.range_min
        range_max = msg.range_max

        mean_range_rad = (MEAN_RANGE * 2 * np.pi)/360 #Degree to Radian 
        step_angle = int(mean_range_rad/msg.angle_increment) #Translates the range we want into the numebr of indexes to navigate
        
        distances = msg.ranges
        nbr_distances = len(distances)
        main_directions_angles = int(nbr_distances/NBR_SENSORS)

        sensor_positions = []
        for i in range(0,NBR_SENSORS):
            sensor_positions.append(i*main_directions_angles)

        np_distances = np.array(distances)

        np_distances = np.nan_to_num(np_distances, posinf=range_max, neginf=range_min)

        sensor_distances = []

        for sensor_pos in sensor_positions:
            indices = np.arange(sensor_pos - step_angle, sensor_pos + step_angle)
            
            wrapped_indices = indices % nbr_distances
            
            sensor_distances.append(np_distances[wrapped_indices])

        self.message.data = [float(np.mean(dist)) for dist in sensor_distances]
        self.publisher_.publish(self.message)


def main(args=None):
    try:
        rclpy.init()
        node = ldsDistanceNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()

