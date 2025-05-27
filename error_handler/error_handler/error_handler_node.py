import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float64MultiArray


class ErrorHandlerNode(Node):

    def __init__(self):
        super().__init__('error_handler')

        self.controll_state = None
        self.error_counter = 0

        self.color_names = {
            1.0: "red",
            2.0: "yellow",
            3.0: "blue",
            4.0: "green"
        }

        self.controll_state_sub = self.create_subscription(
            Int32,
            'controll_state',
            self.controll_state_callback,
            10
        )

        self.detected_cubes_sub = self.create_subscription(
            Float64MultiArray,
            'detected_cubes',
            self.detected_cubes_callback,
            10
        )

        self.controll_state_pub = self.create_publisher(
            Int32,
            'controll_state',
            10
        )

    def controll_state_callback(self, msg):
        self.controll_state = msg.data

    def detected_cubes_callback(self, msg):
        if self.controll_state != 0:
            return

        data = msg.data
        i = 0
        counts = {
            "red": 0,
            "yellow": 0,
            "blue": 0,
            "green": 0,
            "total": 0
        }
        
        while i < len(data):
            color_code = data[i]
            i += 3
            
            color_name = self.color_names.get(color_code, "unknown")
            counts[color_name] += 1
            counts["total"] += 1
             
        if self.error_counter >= 3:
            new_state = 4
        elif counts['total'] == 3:
            new_state = 1
        else:
            new_state = 2
            self.error_counter += 1
        out_msg = Int32()
        out_msg.data = new_state
        self.controll_state_pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    error_handler = ErrorHandlerNode()
    rclpy.spin(error_handler)
    error_handler.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
