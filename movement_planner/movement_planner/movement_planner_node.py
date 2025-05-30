import rclpy
import json
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# Last inn konfigurasjonsfil
with open('config.json') as file:
    data = json.load(file)

class MovementPlannerNode(Node):

    def __init__(self):
        super().__init__('movement_planner')

        # Kart for å oversette fargekoder til navn
        self.color_names = {
            1.0: "red",
            2.0: "yellow",
            3.0: "blue",
            4.0: "green"
        }

        # Ønsket rekkefølge på kuber, hentet fra config.json 
        self.cube_order = [data["cube1"], data["cube2"], data["cube3"]]
        self.z = data["z"] # Z-høyde å bruke for alle posisjoner
        self.debug_mode = data["debug_mode"] # Debug flagg

        # ROS-abonnement og publisher
        self.detected_cubes_sub = self.create_subscription(Float64MultiArray,'detected_cubes',self.detected_cubes_callback,10)
        self.planned_pos_pub = self.create_publisher(Float64MultiArray,'planned_pos',10)

    # Behandler innkommende kubeposisjoner fra fargegjenkjenning
    def detected_cubes_callback(self, msg):
        data = msg.data
        i = 0
        cube_positions = {}

        # Parse hver kube: (fargekode, x, y)
        while i < len(data):
            color_code = data[i]
            x = data[i+1]
            y = data[i+2]
            color_name = self.color_names.get(color_code, "unknown")
            cube_positions[color_name] = (x, y)
            i += 3

        # Sjekk at ALLE ønskede kuber er funnet før publisering
        if all(color in cube_positions for color in self.cube_order):
            positions = []
            for color in self.cube_order:
                x, y = cube_positions[color]
                positions.extend([x, y, self.z]) # Legg til Z-høyde
            out_msg = Float64MultiArray()
            out_msg.data = positions
            self.planned_pos_pub.publish(out_msg)


7
def main(args=None):
    rclpy.init(args=args)
    movementPlanner = MovementPlannerNode()
    rclpy.spin(movementPlanner)
    movementPlanner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
