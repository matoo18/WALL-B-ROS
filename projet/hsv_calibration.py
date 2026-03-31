import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

def null(x):
    pass

class HSVCalibration(Node):
    def __init__(self):
        super().__init__('hsv_calibration')
        
        # on s'abonne à la caméra pour avoir l'image
        self.subscription = self.create_subscription(CompressedImage,'/image_raw/compressed',self.image_callback,10)
        
        # création des trackbars
        cv2.namedWindow('Trackbars')
        cv2.createTrackbar('H_min', 'Trackbars', 0, 179, null)
        cv2.createTrackbar('H_max', 'Trackbars', 179, 179, null)
        cv2.createTrackbar('S_min', 'Trackbars', 0, 255, null)
        cv2.createTrackbar('S_max', 'Trackbars', 255, 255, null)
        cv2.createTrackbar('V_min', 'Trackbars', 0, 255, null)
        cv2.createTrackbar('V_max', 'Trackbars', 255, 255, null)

    def image_callback(self, msg):
        # conversion de l'image pour le traitement
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if cv_image is None:
            return

        # on passe l'image en HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # récupération des valeurs des trackbars (valeurs qu'on injectera ensuite dans les masques)
        h_min = cv2.getTrackbarPos('H_min', 'Trackbars')
        h_max = cv2.getTrackbarPos('H_max', 'Trackbars')
        s_min = cv2.getTrackbarPos('S_min', 'Trackbars')
        s_max = cv2.getTrackbarPos('S_max', 'Trackbars')
        v_min = cv2.getTrackbarPos('V_min', 'Trackbars')
        v_max = cv2.getTrackbarPos('V_max', 'Trackbars')

        # on déf les limites avec les valeurs récup au dessus
        limite_inf = np.array([h_min, s_min, v_min])
        limite_sup = np.array([h_max, s_max, v_max])

        # inRange nous retourne un masque en noir et blanc dont les pixels blancs sont ceux dont la teinte HSV 
        # est comprise entre les bornes inf et sup et les pixels noirs ceux qui ne le sont pas.
        masque = cv2.inRange(hsv, limite_inf, limite_sup)
        # avec cette dernière fonction, on demande de ne faire ressortir que les pixels à la fois présent sur 
        # l'image d'origine ET ceux activés à 1 sur le masque.
        resultat = cv2.bitwise_and(cv_image, cv_image, mask=masque)

        cv2.imshow('Original', cv_image)
        cv2.imshow('Masque', masque)
        cv2.imshow('Resultat', resultat)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = HSVCalibration()
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