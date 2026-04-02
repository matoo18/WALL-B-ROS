import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist 
from std_msgs.msg import Float32MultiArray
import numpy as np

# à revoir car il faut qu'il prenne le dessus sur le noeud de suivi qui envoie des commandes plus régulièrement.

class EMStop(Node):
    def __init__(self):
        super().__init__('em_stop')
        # je m'abonne au topic /lds_means pour pouvoir récup les valeurs du tableau ranges
        self.subscription = self.create_subscription(Float32MultiArray,'/lds_means',self.lds_means_callback,10)

        # on va publier sur cmd_vel l'em stop donc il faut le publisher
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # param qu'on peut régler pour ajuster la distance d'arret d'urgence (1m par défaut)
        self.declare_parameter('em_dist', 1.0)
        self.stopped = False

    def lds_means_callback(self, msg):
        dist = np.array(msg.data)

        em_dist = self.get_parameter('em_dist').get_parameter_value().double_value
        
        if (dist <= em_dist).any():
            self.publisher_.publish(Twist())
            self.stopped = True


def main(args=None):
    rclpy.init(args=args)

    em_stop_publisher = EMStop()

    rclpy.spin(em_stop_publisher)

    em_stop_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
