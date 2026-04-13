import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import math

class VisionGoalNode(Node):
    def __init__(self):
        super().__init__('vision_goal_node')
        
        self.image_subscription = self.create_subscription(CompressedImage, '/image_raw/compressed', self.image_callback, 10)
        self.target_pub = self.create_publisher(Point, '/soccer_target', 10)
        
        # ==========================================================
        # On force la création d'une seule fenêtre en mémoire !
        # ==========================================================
        cv2.namedWindow("Vision Foot", cv2.WINDOW_NORMAL)

        # PARAMÈTRES OPTIQUES (Pour passer des pixels aux mètres)
        self.focale_camera = 330.0 
        self.rayon_reel_balle = 0.05 

        # COULEURS DES OBJETS D'INTÉRETS
        self.balle_hsv_bas = np.array([30, 60, 0])
        self.balle_hsv_haut = np.array([40, 255, 255])
        
        self.bas_hsv_but1 = np.array([0, 70, 50])
        self.haut_hsv_but1 = np.array([10, 255, 255])
        self.bas_hsv_but2 = np.array([170, 70, 50])
        self.haut_hsv_but2 = np.array([180, 255, 255])

    def masque_creation(self, crop_image):
        hsv_image = cv2.cvtColor(crop_image, cv2.COLOR_BGR2HSV)

        masque_balle = cv2.inRange(hsv_image, self.balle_hsv_bas, self.balle_hsv_haut)
        kernel_balle = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        masque_balle = cv2.morphologyEx(masque_balle, cv2.MORPH_OPEN, kernel_balle)

        masque_but = cv2.bitwise_or(cv2.inRange(hsv_image, self.bas_hsv_but1, self.haut_hsv_but1), 
                                  cv2.inRange(hsv_image, self.bas_hsv_but2, self.haut_hsv_but2))
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (50, 5))

        masque_balle = cv2.morphologyEx(masque_balle, cv2.MORPH_CLOSE, kernel_balle)
        masque_balle = cv2.morphologyEx(masque_balle, cv2.MORPH_OPEN, kernel_balle)

        masque_but = cv2.morphologyEx(masque_but, cv2.MORPH_CLOSE, kernel)
        masque_but = cv2.morphologyEx(masque_but, cv2.MORPH_OPEN, kernel)
        return masque_but, masque_balle

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if cv_image is None:
            return

        height, width, _ = cv_image.shape
        centre_camera_x = width / 2.0

        masque_but, masque_balle = self.masque_creation(cv_image)

        # 1. POSITION BALLE
        contours_balle, _ = cv2.findContours(masque_balle, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        balle_trouvee = False
        balle_x, balle_r = 0, 0

        if len(contours_balle) > 0:
            c_balle = max(contours_balle, key=cv2.contourArea)
            if cv2.contourArea(c_balle) > 10:
                ((x, y), r) = cv2.minEnclosingCircle(c_balle)
                balle_x = int(x)
                balle_r = float(r)
                balle_trouvee = True
                cv2.circle(cv_image, (balle_x, int(y)), int(r), (0, 255, 255), 2)

        # 2. POSITION BUT
        contours_but, _ = cv2.findContours(masque_but, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        but_trouve = False
        but_x = 0

        if len(contours_but) > 0:
            contours_but = sorted(contours_but, key=cv2.contourArea, reverse=True)
            for c_but in contours_but:
                if cv2.contourArea(c_but) > 100: 
                    bx, by, bw, bh = cv2.boundingRect(c_but)
                    if bh == 0: continue 
                    ratio_forme = float(bw) / float(bh)

                    if ratio_forme > 1.2:
                        but_x = int(bx + (bw / 2.0))
                        but_trouve = True
                        cv2.rectangle(cv_image, (bx, by), (bx+bw, by+bh), (0, 255, 0), 3) 
                        cv2.putText(cv_image, "BUT VALIDE", (bx, by - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        break 
                    else:
                        cv2.rectangle(cv_image, (bx, by), (bx+bw, by+bh), (0, 0, 255), 1) 
                        cv2.putText(cv_image, "REJETE (Faux positif)", (bx, by - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

        # 3. CONVERSION PIXELS -> MÈTRES
        target_msg = Point()
        etat_txt = "RECHERCHE BALLE"

        if balle_trouvee and balle_r > 0:
            distance_x = (self.focale_camera * self.rayon_reel_balle) / balle_r
            dx_pixel = centre_camera_x - balle_x 
            distance_y = distance_x * (dx_pixel / self.focale_camera)
            
            angle_but = 0.0
            if but_trouve:
                dx_but_pixel = centre_camera_x - but_x
                angle_but = math.atan(dx_but_pixel / self.focale_camera)
                etat_txt = f"CIBLE OK (Dist: {distance_x:.2f}m)"
            else:
                etat_txt = "BALLE VUE (PAS DE BUT)"

            target_msg.x = float(distance_x)  
            target_msg.y = float(distance_y)  
            target_msg.z = float(angle_but)   
            
        else:
            target_msg.x = -1.0 
            target_msg.y = 0.0
            target_msg.z = 0.0

        self.target_pub.publish(target_msg)

        cv2.putText(cv_image, etat_txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.imshow("Vision Foot", cv_image)
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