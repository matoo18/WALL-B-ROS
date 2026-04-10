import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class But(Node):
    def __init__(self):
        super().__init__('but')
        
        self.image_subscription = self.create_subscription(CompressedImage, '/image_raw/compressed', self.image_callback, 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # ==========================================================
        # COULEURS DES OBJETS D'INTÉRETS
        # ==========================================================

        # Balle jaune
        self.balle_hsv_bas = np.array([30, 60, 0])
        self.balle_hsv_haut = np.array([40, 255, 255])
        
        # But rouge 
        self.bas_hsv_but1 = np.array([0, 70, 50])
        self.haut_hsv_but1 = np.array([10, 255, 255])
        self.bas_hsv_but2 = np.array([170, 70, 50])
        self.haut_hsv_but2 = np.array([180, 255, 255])
