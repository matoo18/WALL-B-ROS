import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class VisionGoalNode(Node):
    def __init__(self):
        super().__init__('vision_goal_node')
        
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

    def masque_creation(self, crop_image):
        # on passe en hsv meilleure saturation
        hsv_image = cv2.cvtColor(crop_image, cv2.COLOR_BGR2HSV)

        # création des masques, on a utilisé le noeud hsv_calibration avec les trackbars pour trouver les valeurs
        masque_balle = cv2.inRange(hsv_image, self.balle_hsv_bas, self.balle_hsv_haut)

        kernel_balle = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        masque_balle = cv2.morphologyEx(masque_balle, cv2.MORPH_OPEN, kernel_balle)

        # on fusionne les 2 masques avec bitwise_or qui garde tous les pixels des 2 masques 
        masque_but = cv2.bitwise_or(cv2.inRange(hsv_image, self.bas_hsv_but1, self.haut_hsv_but1), 
                                  cv2.inRange(hsv_image, self.bas_hsv_but2, self.haut_hsv_but2))

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (50, 5))

        # création des masques finies, on fait une fermeture suivie d'une ouverture pour chacun des masque 
        # pour que les points noirs dans les lignes deviennent blancs et que les points blancs dans hors des lignes
        # deviennent noirs.
        masque_balle = cv2.morphologyEx(masque_balle, cv2.MORPH_CLOSE, kernel_balle)
        masque_balle = cv2.morphologyEx(masque_balle, cv2.MORPH_OPEN, kernel_balle)

        masque_but = cv2.morphologyEx(masque_but, cv2.MORPH_CLOSE, kernel)
        masque_but = cv2.morphologyEx(masque_but, cv2.MORPH_OPEN, kernel)
        return masque_but, masque_balle

    def image_callback(self, msg):
        # =================================================================
        # ACQUISITION ET PRÉTRAITEMENT DE L'IMAGE
        # =================================================================

        # conversion pour la lecture
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if cv_image is None:
            return

        # on prend les dimensions qui permettront de croper l'image pour ne garder que le bas de celle-ci
        # pour ne pas que l'algo ne prenne en compte les lignes lointaines trop tot
        height, width, _ = cv_image.shape

        masque_but, masque_balle = self.masque_creation(cv_image)

        masque_total = cv2.bitwise_or(masque_but, masque_balle)
        cv2.imshow("masque", masque_total)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = VisionGoalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

