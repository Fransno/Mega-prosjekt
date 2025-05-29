import rclpy
import json
from rclpy.node import Node
from std_msgs.msg import Int32, Float64MultiArray
from collections import Counter

# Laster inn ønsket kubeoppsett og systemparametere fra config-filen
with open('config.json') as file:
    data = json.load(file)

class ErrorHandlerNode(Node):

    def __init__(self):
        super().__init__('error_handler')

        # Intern tilstand
        self.controll_state = None # Gjeldende kontrolltilstand
        self.error_counter = 0 # Teller hvor mange ganger det har feilet

        # Teller hvor mange ganger hver farge skal oppdages
        self.wanted_counts = Counter([data["cube1"], data["cube2"], data["cube3"]])
        self.debug_mode = data["debug_mode"]

        # Fargekoder fra deteksjon til tekstnavn
        self.color_names = {
            1.0: "red",
            2.0: "yellow",
            3.0: "blue",
            4.0: "green"
        }

        # Abonnementer
        self.detected_cubes_sub = self.create_subscription(Float64MultiArray,'detected_cubes',self.detected_cubes_callback,10)
        self.controll_state_sub = self.create_subscription(Int32,'controll_state',self.controll_state_callback,10)

        # Publisher
        self.controll_state_pub = self.create_publisher(Int32,'controll_state',10)

        # Initialiser med ønsket starttilstand (f.eks. 3 for hjemposisjon)
        initial_msg = Int32()
        initial_msg.data = data["controll_state"]
        self.controll_state_pub.publish(initial_msg)
        

    def controll_state_callback(self, msg):
        """ Oppdaterer intern kontrolltilstand basert på meldinger fra andre noder. """
        self.controll_state = msg.data
        if self.debug_mode:
            self.get_logger().info(f'Detected controll state: {self.controll_state}')

    def detected_cubes_callback(self, msg):
        """ Behandler innkommende deteksjonsdata og publiserer ny kontrolltilstand. """
        if self.controll_state != 0:
            return # Bare aktiv i state 0

        # Tell opp antall av hver farge som er observert
        i = 0
        counts = {
            "red": 0,
            "yellow": 0,
            "blue": 0,
            "green": 0
        }
        
        while i < len(msg.data):
            color_code = msg.data[i]
            i += 3 # Hopp til neste kube (hver kube: fargekode, x, y)
            color_name = self.color_names.get(color_code, "unknown")
            counts[color_name] += 1

        # Sjekk hvilken tilstand som skal publiseres
        if self.error_counter >= 3:
            new_state = 4 # Abort
            if self.debug_mode:
                self.get_logger().info(f'Counted 3 errors: {msg.data}')
        elif all(counts[color] >= self.wanted_counts[color] for color in self.wanted_counts):
            new_state = 1 # Alle ønskede kuber funnet
            if self.debug_mode:
                self.get_logger().info(f'Cubes good with: {msg.data}')
        else:
            new_state = 2 # Ikke alle kuber funnet, prøv igjen
            self.error_counter += 1
            if self.debug_mode:
                self.get_logger().info(f'Error: {msg.data}')

        # Publiser ny tilstand
        out_msg = Int32()
        out_msg.data = new_state
        self.controll_state_pub.publish(out_msg)
        if self.debug_mode:
            self.get_logger().info(f'Published controll state: {new_state}')


def main(args=None):
    rclpy.init(args=args)
    error_handler = ErrorHandlerNode()
    rclpy.spin(error_handler)
    error_handler.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
