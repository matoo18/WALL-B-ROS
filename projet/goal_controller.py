import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Int32
import math

class SoccerController(Node):
    def __init__(self):
        super().__init__('soccer_controller')
        self.target_sub = self.create_subscription(Point, '/soccer_target', self.target_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        

    def target_callback(self, msg):
        ball_x = msg.x
        ball_y = msg.y
        angle_but = msg.z 
        
        twist = Twist()

        # =================================================================
        # ÉTAT 0 : PERTE DE LA BALLE
        # =================================================================
        if ball_x < 0:
            self.get_logger().info("RECHERCHE BALLE (Je tourne sur moi-même)")
            twist.angular.z = 0.5 
            self.cmd_pub.publish(twist)
            return

        dist_to_ball = math.hypot(ball_x, ball_y)

        # =================================================================
        # ÉTAT 1 : APPROCHE (Si on est à plus de 45cm)
        # =================================================================
        if dist_to_ball > 0.45:
            self.get_logger().info("APPROCHE : Je fonce vers la balle")
            twist.linear.x = 0.15
            twist.angular.z = float(ball_y * 2.5) # On vise la balle plein centre
            self.cmd_pub.publish(twist)
            return

        # =================================================================
        # ÉTAT 2 : ORBITING / ALIGNEMENT (On est près de la balle)
        # =================================================================
        # Si l'angle du but est grand (on n'est pas aligné)
        if abs(angle_but) > 0.15: 
            
            # On décide de quel côté tourner.
            # Si le but est à gauche (angle positif), on doit tourner autour par la droite.
            # On va donc forcer le robot à garder la balle sur la GAUCHE de son écran (+0.15m).
            if angle_but > 0:
                consigne_y = 0.15
                etat = "ORBITING DROITE"
            else:
                consigne_y = -0.15
                etat = "ORBITING GAUCHE"

            # 1. On avance pour créer le mouvement
            twist.linear.x = 0.08
            
            # 2. Le volant (PID) : On tourne pour maintenir la balle à la consigne (sur le bord de l'écran)
            erreur_volant = ball_y - consigne_y
            twist.angular.z = float(erreur_volant * 4.0)

            self.get_logger().info(f"{etat} | Je décale la balle | Angle But: {math.degrees(angle_but):.0f}°")

        # =================================================================
        # ÉTAT 3 : LA FRAPPE (Aligné et près !)
        # =================================================================
        else:
            self.get_logger().info("🎯 ALIGNÉ ! FRAPPE !!!")
            twist.linear.x = 0.35 # Coup de boost
            twist.angular.z = float(ball_y * 2.5) # On maintient la balle au centre pendant le tir

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = SoccerController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()