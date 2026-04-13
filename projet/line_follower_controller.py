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

        self.cible_buffer = []
        self.frames_delay = 5

    def vision_callback(self, msg):
        twist = Twist()
    
        cible_x = msg.x
        cam_centre = msg.y
        is_tracking = msg.z
        
        if is_tracking == 0.0:
            # si pas assez de points pour tracer une trajectoire
            twist.linear.x = 0.05
            twist.angular.z = 0.0

            self.cible_buffer.clear() 
            self.publisher_.publish(twist)
            return
        
        self.cible_buffer.append(msg.x)
        
        if len(self.cible_buffer) < self.frames_delay:
            twist.linear.x = 0.05
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            return
            
        cible_x_retardee = self.cible_buffer.pop(0)
            
        error_x = cam_centre - cible_x_retardee
        kp_x = 0.005
        angular = float(error_x * kp_x)

        # ralentissement de la vitesse linéaire quand il tourne sinon il partirait dans les choux
        twist.linear.x = max(0.05, 0.15 - 0.5 * abs(angular))
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