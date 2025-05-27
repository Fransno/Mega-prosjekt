import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class MovementPlannerNode(Node):

    def __init__(self):
        super().__init__('movement_planner')

        self.color_names = {
            1.0: "red",
            2.0: "yellow",
            3.0: "blue",
            4.0: "green"
        }

        self.order = {
            1.0: "red",
            2.0: "yellow",
            3.0: "blue",
        }

        self.detected_cubes_sub = self.create_subscription(
            Float64MultiArray,
            'detected_cubes',
            self.detected_cubes_callback,
            10
        )

        self.planned_pos_pub = self.create_publisher(
            Float64MultiArray,
            'planned_pos',
            10
        )


    def detected_cubes_callback(self, msg):
        data = msg.data
        i = 0
        # Map color name to (x, y)
        cube_positions = {}
        while i < len(data):
            color_code = data[i]
            x = data[i+1]
            y = data[i+2]
            color_name = self.color_names.get(color_code, "unknown")
            if color_name in self.order.values():
                cube_positions[color_name] = (x, y)
            i += 3

        # Only publish if all three colors are present
        if all(color in cube_positions for color in self.order.values()):
            positions = []
            for color_code in sorted(self.order.keys()):
                color_name = self.order[color_code]
                x, y = cube_positions[color_name]
                positions.extend([x, y, 10.0])
            out_msg = Float64MultiArray()
            out_msg.data = positions
            self.planned_pos_pub.publish(out_msg)



def main(args=None):
    rclpy.init(args=args)
    movementPlanner = MovementPlannerNode()
    rclpy.spin(movementPlanner)
    movementPlanner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()