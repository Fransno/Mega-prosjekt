import rclpy
import json
from rclpy.node import Node
from std_msgs.msg import Int32, Float64MultiArray

# Load JSON
with open('config.json') as file:
    data = json.load(file)

class MovementControllerNode(Node):

    def __init__(self):
        super().__init__('movement_controller')

        self.controll_state = None
        self.gathered_pos = False
        self.counter = 0

        self.home_pos = Float64MultiArray()
        self.home_pos.data = data["home_position"]
        self.zoom_value = data["zoom_value"]
        self.debug_mode = data["debug_mode"]

        self.pos1 = Float64MultiArray()
        self.pos2 = Float64MultiArray()
        self.pos3 = Float64MultiArray()

        self.controll_state_sub = self.create_subscription(Int32,'controll_state',self.controll_state_callback,10)
        self.controll_state_pub = self.create_publisher(Int32,'controll_state',10)
        self.movement_state_sub = self.create_subscription(Int32,'movement_state',self.movement_state_callback,10)
        self.movement_state_pub = self.create_publisher(Int32,'movement_state',10)
        self.planned_pos_sub = self.create_subscription(Float64MultiArray,'planned_pos',self.planned_pos_callback,10)
        self.next_pos_pub = self.create_publisher(Float64MultiArray,'next_pos',10)

    def controll_state_callback(self, msg):
        if self.debug_mode:
            self.get_logger().info(f'Detected controll state: {msg.data}')
        self.controll_state = msg.data
        if msg.data == 2:
            self.movement_state_pub.publish(Int32(data=0))

    def movement_state_callback(self, msg):
        if self.debug_mode:
            self.get_logger().info(f'Detected movement_state: {msg.data}')
        if self.controll_state == 3:
            if msg.data == 0:
                self.next_pos_pub.publish(self.home_pos)
                self.movement_state_pub.publish(Int32(data=1))
                if self.debug_mode:
                    self.get_logger().info(f'Moving to home: {self.home_pos.data}')
                    self.get_logger().info(f'Published movement_state: 1')
            elif msg.data == 2:
                self.controll_state_pub.publish(Int32(data=0))
                self.movement_state_pub.publish(Int32(data=0))
                if self.debug_mode:
                    self.get_logger().info('Published controll_state: 0')
                    self.get_logger().info('Published movement_state: 0')

        elif self.controll_state == 2:
            if msg.data == 0:
                self.home_pos.data[2] += self.zoom_value
                self.next_pos_pub.publish(self.home_pos)
                self.movement_state_pub.publish(Int32(data=1))
                if self.debug_mode:
                    self.get_logger().info(f'Published zoomed out next_pos: {self.home_pos.data}')
                    self.get_logger().info(f'Published movement_state: 1')
            elif msg.data == 2:
                self.controll_state_pub.publish(Int32(data=0))
                self.movement_state_pub.publish(Int32(data=0))
                if self.debug_mode:
                    self.get_logger().info('Published controll_state: 0')
                    self.get_logger().info('Published movement_state: 0')

        elif self.controll_state == 1:
            if self.gathered_pos:
                if msg.data == 0:
                    if self.counter == 0:
                        self.next_pos_pub.publish(self.pos1)
                        self.movement_state_pub.publish(Int32(data=1))
                        self.counter += 1
                        if self.debug_mode:
                            self.get_logger().info(f'Published first next_pos: {self.pos1.data}')
                            self.get_logger().info(f'Published movement_state: 1')
                    elif self.counter == 1:
                        self.next_pos_pub.publish(self.pos2)
                        self.movement_state_pub.publish(Int32(data=1))
                        self.counter += 1
                        if self.debug_mode:
                            self.get_logger().info(f'Published second next_pos: {self.pos2.data}')
                            self.get_logger().info(f'Published movement_state: 1')
                    elif self.counter == 2:
                        self.next_pos_pub.publish(self.pos3)
                        self.movement_state_pub.publish(Int32(data=1))
                        self.counter += 1
                        if self.debug_mode:
                            self.get_logger().info(f'Published third next_pos: {self.pos3.data}')
                            self.get_logger().info(f'Published movement_state: 1')
                elif msg.data == 2:
                    self.movement_state_pub.publish(Int32(data=0))
                    if self.debug_mode:
                        self.get_logger().info('Published movement_state: 0')

    def planned_pos_callback(self, msg): 
        if self.controll_state != 1:
            return
        
        if not self.gathered_pos:
            if len(msg.data) == 9:
                if self.debug_mode:
                    self.get_logger().info(f'Saved positions: {msg.data}')
                self.pos1.data = msg.data[0:3]
                self.pos2.data = msg.data[3:6]
                self.pos3.data = msg.data[6:9]
                self.gathered_pos = True
                self.movement_state_pub.publish(Int32(data=0))


def main(args=None):
    rclpy.init(args=args)
    movementController = MovementControllerNode()
    rclpy.spin(movementController)
    movementController.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
