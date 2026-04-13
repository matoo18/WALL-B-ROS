import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32MultiArray
import cv2
import numpy as np
import threading

def null(x):
    pass

class HSVCalibration(Node):
    def __init__(self):
        super().__init__('hsv_calibration')
        
        # on s'abonne à la caméra pour avoir l'image
        self.subscription = self.create_subscription(CompressedImage,'camera/image_raw/compressed',self.image_callback,10)
        self.hsv_pub = self.create_publisher(Int32MultiArray,'/live_hsv',10)

        self.current_image = None
        
        # création des VERT
        cv2.namedWindow('VERT')
        cv2.createTrackbar('H_min', 'VERT', 35, 179, null)
        cv2.createTrackbar('H_max', 'VERT', 107, 179, null)
        cv2.createTrackbar('S_min', 'VERT', 22, 255, null)
        cv2.createTrackbar('S_max', 'VERT', 255, 255, null)
        cv2.createTrackbar('V_min', 'VERT', 140, 255, null)
        cv2.createTrackbar('V_max', 'VERT', 255, 255, null)

        cv2.namedWindow('ROUGE BAS')
        # Pré-réglé pour le rouge bas (0 à 10)
        cv2.createTrackbar('H_min', 'ROUGE BAS', 0, 179, null)
        cv2.createTrackbar('H_max', 'ROUGE BAS', 10, 179, null)
        cv2.createTrackbar('S_min', 'ROUGE BAS', 70, 255, null)
        cv2.createTrackbar('S_max', 'ROUGE BAS', 255, 255, null)
        cv2.createTrackbar('V_min', 'ROUGE BAS', 50, 255, null)
        cv2.createTrackbar('V_max', 'ROUGE BAS', 255, 255, null)

        cv2.namedWindow('ROUGE HAUT')
        # Pré-réglé pour le rouge haut (170 à 179)
        cv2.createTrackbar('H_min', 'ROUGE HAUT', 170, 179, null)
        cv2.createTrackbar('H_max', 'ROUGE HAUT', 179, 179, null)
        cv2.createTrackbar('S_min', 'ROUGE HAUT', 70, 255, null)
        cv2.createTrackbar('S_max', 'ROUGE HAUT', 255, 255, null)
        cv2.createTrackbar('V_min', 'ROUGE HAUT', 50, 255, null)
        cv2.createTrackbar('V_max', 'ROUGE HAUT', 255, 255, null)

        cv2.namedWindow('BLEU')
        # Pré-réglé pour le rouge haut (170 à 179)
        cv2.createTrackbar('H_min', 'BLEU', 90, 179, null)
        cv2.createTrackbar('H_max', 'BLEU', 150, 179, null)
        cv2.createTrackbar('S_min', 'BLEU', 25, 255, null)
        cv2.createTrackbar('S_max', 'BLEU', 255, 255, null)
        cv2.createTrackbar('V_min', 'BLEU', 0, 255, null)
        cv2.createTrackbar('V_max', 'BLEU', 255, 255, null)

    def image_callback(self, msg):
        # Ce callback s'exécute dans le thread ROS. 
        # Il est ultra-rapide : il décode juste l'image et la met en mémoire.
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.current_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def process_and_display(self):
        # Cette fonction sera appelée en boucle dans le thread principal
        if self.current_image is None:
            return

        # on passe l'image en HSV
        cv_image = self.current_image.copy()
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # récupération des valeurs des VERT (valeurs qu'on injectera ensuite dans les masques)
        h_min = cv2.getTrackbarPos('H_min', 'VERT')
        h_max = cv2.getTrackbarPos('H_max', 'VERT')
        s_min = cv2.getTrackbarPos('S_min', 'VERT')
        s_max = cv2.getTrackbarPos('S_max', 'VERT')
        v_min = cv2.getTrackbarPos('V_min', 'VERT')
        v_max = cv2.getTrackbarPos('V_max', 'VERT')

        h_min_1 = cv2.getTrackbarPos('H_min', 'ROUGE BAS')
        h_max_1 = cv2.getTrackbarPos('H_max', 'ROUGE BAS')
        s_min_1 = cv2.getTrackbarPos('S_min', 'ROUGE BAS')
        s_max_1 = cv2.getTrackbarPos('S_max', 'ROUGE BAS')
        v_min_1 = cv2.getTrackbarPos('V_min', 'ROUGE BAS')
        v_max_1 = cv2.getTrackbarPos('V_max', 'ROUGE BAS')

        # LECTURE DE LA PLAGE 2
        h_min_2 = cv2.getTrackbarPos('H_min', 'ROUGE HAUT')
        h_max_2 = cv2.getTrackbarPos('H_max', 'ROUGE HAUT')
        s_min_2 = cv2.getTrackbarPos('S_min', 'ROUGE HAUT')
        s_max_2 = cv2.getTrackbarPos('S_max', 'ROUGE HAUT')
        v_min_2 = cv2.getTrackbarPos('V_min', 'ROUGE HAUT')
        v_max_2 = cv2.getTrackbarPos('V_max', 'ROUGE HAUT')

        h_min_b = cv2.getTrackbarPos('H_min', 'BLEU')
        h_max_b = cv2.getTrackbarPos('H_max', 'BLEU')
        s_min_b = cv2.getTrackbarPos('S_min', 'BLEU')
        s_max_b = cv2.getTrackbarPos('S_max', 'BLEU')
        v_min_b = cv2.getTrackbarPos('V_min', 'BLEU')
        v_max_b = cv2.getTrackbarPos('V_max', 'BLEU')


        msg_out = Int32MultiArray()
        msg_out.data = [
            # Plage vert
            int(h_min), int(s_min), int(v_min), int(h_max), int(s_max), int(v_max),
            # Plage Rouge Bas
            int(h_min_1), int(s_min_1), int(v_min_1), int(h_max_1), int(s_max_1), int(v_max_1),
            # Plage Rouge Haut
            int(h_min_2), int(s_min_2), int(v_min_2), int(h_max_2), int(s_max_2), int(v_max_2),

            int(h_min_b), int(s_min_b), int(v_min_b), int(h_max_b), int(s_max_b), int(v_max_b)
        ]
        self.hsv_pub.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = HSVCalibration()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        while rclpy.ok():
            node.process_and_display()
            # On force le rafraîchissement des trackbars toutes les 30 millisecondes
            cv2.waitKey(30)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()