import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class LineFollowing(Node):
    def __init__(self):
        super().__init__('LineFollowing')
        # on s'abonne au topic qui publie les images compressées reçues de la caméra
        self.image_subscription = self.create_subscription(CompressedImage,'/image_raw/compressed',self.image_callback,10)

        # Cration du publisher pour pouvoir publier les vitesses sur le topic /cmd_vel
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # attributs de mémoire qui seront utiles lorsqu'une ligne disparaitra du champs de vision de la caméra
        self.memoire_ligne_width = 300.0
        self.last_cible_x = 160.0

        # on déclare un paramètre pour pouvoir choisir dans quel sens prendre le rond-point
        self.declare_parameter('roundabout_dir', 'right')

    def get_line_centres(self, masque, y_raies):
        centres = {}
        for y in y_raies:
            # je regarde la ligne y dans toute sa longueur
            l = masque[y, :]
            # là où les valeurs dans la liste sont >0 ça veut dire qu'il y a du vert (ou rouge en fonction du masque doné)
            # [0] parce que np.where renvoie un tableau bizarre.
            indices = np.where(l > 0)[0]
            # s'il n'y a pas de pixels blancs sur la ligne alors on passe sinon on calcule la moyenne de la position des
            # pixels et on met ça dans la liste des centres qui permettront de tracer la trajectoire à suivre
            if len(indices) > 0:
                centres[y] = int(np.mean(indices))
        return centres

    def image_callback(self, msg):
        # conversion pour la lecture
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if cv_image is None:
            return

        # on prend les dimensions qui permettront de croper l'image pour ne garder que le bas de celle-ci
        # pour ne pas que l'algo ne prenne en compte les lignes lointaines trop tot
        height, width, _ = cv_image.shape

        # jeu de mot drole
        crop_top = int(height * 0.5) # on peut jouer sur 0.5 pour voir à quel point on peut anticiper
        crop_image = cv_image[crop_top:height, :]

        # on passe en hsv meilleure saturation
        hsv_image = cv2.cvtColor(crop_image, cv2.COLOR_BGR2HSV)

        # création des masques, on a utilisé le noeud hsv_calibration avec les trackbars pour trouver les valeurs
        bas_vert = np.array([40, 30, 10])
        haut_vert = np.array([90, 255, 255])
        masque_vert = cv2.inRange(hsv_image, bas_vert, haut_vert)

        # pour des raisons de mémoire, les valeurs demandées par la fonction de cv2 doivent etre contenues dans 
        # 1 octet donc pour la saturation et la luminosité c'est carré mais pour le Hue on divise simplement par 2
        # mais du coup la zone de rouge se retrouve coupée en 2 d'où la nécessité des 2 masques.
        bas_rouge1 = np.array([0, 70, 50])
        haut_rouge1 = np.array([10, 255, 255])
        bas_rouge2 = np.array([170, 70, 50])
        haut_rouge2 = np.array([180, 255, 255])

        # on fusionne les 2 masques avec bitwise_or qui garde tous les pixels des 2 masques 
        masque_rouge = cv2.bitwise_or(cv2.inRange(hsv_image, bas_rouge1, haut_rouge1), 
                                  cv2.inRange(hsv_image, bas_rouge2, haut_rouge2))

        # nous donne un kernel en forme de cercle pour que ce soit plus smooth sur les bords qu'un kernel rectangulaire.
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

        # création des masques finies, on fait une fermeture suivie d'une ouverture pour chacun des masque 
        # pour que les points noirs dans les lignes deviennent blancs et que les points blancs dans hors des lignes
        # deviennent noirs.
        masque_vert = cv2.morphologyEx(masque_vert, cv2.MORPH_CLOSE, kernel)
        masque_vert = cv2.morphologyEx(masque_vert, cv2.MORPH_OPEN, kernel)

        masque_rouge = cv2.morphologyEx(masque_rouge, cv2.MORPH_CLOSE, kernel)
        masque_rouge = cv2.morphologyEx(masque_rouge, cv2.MORPH_OPEN, kernel)

        # on récup les nouvelles height et width de l'image cropée
        crop_h, crop_w = crop_image.shape[:2]

        # utilisation de la fonciton get_line_centres pour récupérer le centre des lignes isolées précédemment.
        # d'abord on créé des raies horizontales d'un pixel de large tous les 10 pixels sur lesquelles on regardera 
        # s'il y a des points blancs.
        y_raies = range(10, crop_h - 10, 10)

        # on chope le centre du masque vert 
        centre_vert = self.get_line_centres(masque_vert, y_raies)
        centre_rouge = self.get_line_centres(masque_rouge, y_raies)

        chemin_x = []
        chemin_y = []

        direction = self.get_parameter('roundabout_dir').get_parameter_value().string_value

        for y in y_raies:
            # on récup pour chaque raie y la valeur moyenne associée à la clé y dans le dictionnaire récup juste au dessus
            x_vert = centre_vert.get(y)
            x_rouge = centre_rouge.get(y)

            # si les on a des moyennes de positions pour les 2 couleurs, alors
            if x_vert is not None and x_rouge is not None:
                # on check si la ligne rouge est bien à droite de la ligne verte
                if x_rouge > x_vert:
                    alpha = 0.1
                    # inspiré du RL, plutot que de brutalement changer la valeur de la dernière largeur de passage mesurée, 
                    # on fait une sorte de bootstrapping pour faire une sorte de moyenne glissante.
                    self.memoire_ligne_width = alpha * (x_rouge - x_vert) + (1 - alpha) * self.memoire_ligne_width

                vx = (x_vert + x_rouge) / 2.0
                chemin_x.append(vx)
                chemin_y.append(y)

            # Dans le cas où la ligne verte n'est plus visible
            elif x_rouge is not None:
                # on triche en utilisant la mémoire de la largeur de piste pour estimer le centre entre les 2 lignes.
                vx = x_rouge - (self.memoire_ligne_width / 2.0)
                chemin_x.append(vx)
                chemin_y.append(y)

            # pareil si la rouge n'est plus visible
            elif x_vert is not None:
                vx = x_vert + (self.memoire_ligne_width / 2.0)
                chemin_x.append(vx)
                chemin_y.append(y)

        # au cas où l'image est complètement noire, on dit que la cible est la meme que la précédente
        cible_x = self.last_cible_x
        futur_y = int(crop_h * 0.3)

        if len(chemin_y) >= 4:
            # on calcule un polynome de degré 3 qui passe par les points du chemin x,y
            poly = np.polyfit(chemin_y, chemin_x, 3)
            
            # ensuite on demande la valeur de x, la prochaine cible, en y_futur pour l'anticipation 
            cible_x = np.polyval(poly, futur_y)
            self.last_cible_x = cible_x

            # pour l'affichage de ma super ligne à l'écran
            pts = []
            for y in range(0, crop_h, 5):
                x = int(np.polyval(poly, y))
                pts.append([x, y + crop_top])
            pts = np.array(pts, np.int32)
            pts = pts.reshape((-1, 1, 2))
            cv2.polylines(cv_image, [pts], False, (0, 0, 255), 2)
        
        # plan de secours si on a pas assez de points dans le chemin, on prend juste la moyenne des coords de x
        # pour s'orienter grossièrement 
        elif len(chemin_y) > 0:
            cible_x = np.mean(chemin_x)
            self.last_cible_x = cible_x

        # on calcule l'erreur entre le centre de la caméra (là où regarde le robot) et la 
        # cible (là où devrait reagrder le robot)
        cam_centre = crop_w / 2.0
        error_x = cam_centre - cible_x

        twist = Twist()
        kp_x = 0.005
        angular = float(error_x * kp_x)

        if len(chemin_y) == 0:
            # si le robot n'a plus de route on le fait avancer tout doux pour qu'il en retrouve une.
            twist.linear.x = 0.05
        else:
            # s'il a une route alors on prend le max entre sa vitesse sans route et la différence entre sa vitesse 
            # max et le braquage qu'il doit faire. Ceci permet de ralentir sa vitesse linéaire quand il tourne.
            twist.linear.x = max(0.05, 0.15 - 0.5 * abs(angular))

        twist.angular.z = angular
        self.publisher_.publish(twist)

        # un peu de dessin pour la beautéééééé
        cv2.line(cv_image, (0, crop_top), (width, crop_top), (255, 0, 255), 2)
        draw_y = crop_top + futur_y if len(chemin_y) >= 3 else crop_top + int(crop_h/2)
        cv2.circle(cv_image, (int(cible_x), draw_y), 7, (255, 0, 0), -1)

        cv2.imshow("Vue du robot", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowing()
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