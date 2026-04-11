import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_srvs.srv import Trigger
import time

class MaestroNode(Node):
    def __init__(self):
        super().__init__('maestro')

        # le service écoute les appels du client venant de la vision
        self.srv = self.create_service(Trigger, '/next_challenge', self.trigger_callback)

        # ensuite on spamera 10x/sec sur un topic le challenge actuel pour que si on ajoute un noeud en cours de route il soit au courant aussi
        self.state_pub = self.create_publisher(Int32, '/active_state', 10)
        self.timer = self.create_timer(0.1, self.publish_state)

        self.current_challenge = 1
        
        # Check si on a pas déjà reçu la demande
        self.last_franchis = 0.0
        self.cooldown = 5.0  

    def trigger_callback(self, request, response):
        # cette focntion est appelée à chaque fois que le client dans vision appelle le service pour changer de challenge
        current_time = time.time()
        
        # Check si on a pas déjà reçu la demande
        if (current_time - self.last_franchis) > self.cooldown:
            self.current_challenge += 1
            self.last_franchis = current_time
            
            self.get_logger().info(f"Ligne bleue détectée, on passe au challenge {self.current_challenge} ==")
            
            # réponse à renvoyer à vision 
            response.success = True
            response.message = f"Passage au challenge {self.current_challenge}"
        else:
            response.success = False
            response.message = "On a déjà eu l'info ici tqt !"
            
        return response

    def publish_state(self):
        msg = Int32()
        msg.data = self.current_challenge
        self.state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MaestroNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()