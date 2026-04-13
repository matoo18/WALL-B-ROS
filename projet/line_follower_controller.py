import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point

class LineFollowerController(Node):
    def __init__(self):
        super().__init__('line_follower_controller')
        
        # abonnement à la cible calculée par le noeud de vision
        self.vision_subscription = self.create_subscription(Point, '/vision_target', self.vision_callback, 10)
        
        # pour publier les vitesses
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.last_error_x = 0.0 

        # =========================================================
        # LA SOLUTION : LE BUFFER TEMPOREL (FIFO)
        # =========================================================
        self.cible_buffer = []
        
        # C'est ICI que tu vas régler ton offset !
        # 5 frames à 30fps = ~160 millisecondes de décalage.
        # Si le robot coupe encore, augmente ce chiffre (ex: 8).
        # S'il tourne trop tard (dépasse le virage), diminue-le (ex: 3).
        self.frames_delay = 5 

    def vision_callback(self, msg):
        twist = Twist()
    
        cam_centre = msg.y
        is_tracking = msg.z
        
        if is_tracking == 0.0:
            # si pas assez de points pour tracer une trajectoire
            twist.linear.x = 0.05
            twist.angular.z = 0.0
            
            # On vide le buffer de délai, car la piste est perdue !
            self.cible_buffer.clear() 
            self.publisher_.publish(twist)
            return
            
        # =========================================================
        # GESTION DU DÉCALAGE TEMPOREL
        # =========================================================
        # 1. On stocke ce que la caméra voit MAINTENANT
        self.cible_buffer.append(msg.x)
        
        # 2. Tant que le buffer n'est pas rempli, on maintient la ligne droite
        if len(self.cible_buffer) < self.frames_delay:
            twist.linear.x = 0.05
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            return
            
        # 3. On extrait la cible qui a été vue il y a 'frames_delay' images !
        cible_x_retardee = self.cible_buffer.pop(0)

        # =========================================================
        # CALCUL DE L'ERREUR (Avec la cible retardée)
        # =========================================================
        error_x = cam_centre - cible_x_retardee
        kp_x = 0.005
        kd_x = 0.008
        
        derivee = error_x - self.last_error_x
        angular = float(error_x * kp_x + derivee * kd_x)

        self.last_error_x = error_x

        # ralentissement de la vitesse linéaire quand il tourne
        twist.linear.x = max(0.05, 0.09 - 0.5 * abs(angular))
        twist.angular.z = angular

        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()