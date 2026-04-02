import rclpy
from rclpy.node import Node
import click
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import threading
import copy

import numpy as np


class velocityControl(Node):
    def __init__(self):
        super().__init__('lds_distance')

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription_ = self.create_subscription(Float32MultiArray, '/lds_means', self.distance_callback,10)
        self.message = copy.deepcopy(Twist()) 

    def distance_callback(self, msg):
        pass


def main(args=None):
    try:
        rclpy.init()
        node = velocityControl()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()

