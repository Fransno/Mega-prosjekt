import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Int32, Float64MultiArray
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import Constraints, PositionConstraint
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient
import threading


class RobotCommNode(Node):
    def __init__(self):
        super().__init__('robot_comm_node')

        self.next_pos = None

        # sub/pub
        self.next_pos_sub = self.create_subscription(Float64MultiArray, 'next_pos', self.next_pos_callback, 10)
        self.movement_state_sub = self.create_subscription(Int32, 'movement_state', self.movement_state_callback, 10)
        self.movement_state_pub = self.create_publisher(Int32, 'movement_state', 10)
        self.client = ActionClient(self, MoveGroup, '/move_action')

        # Robotkonfigurasjon
        self.group_name = 'ur_manipulator'
        self.frame_id = 'base_link'
        self.link_name = 'tool0'

    def movement_state_callback(self, msg):
        if msg.data != 1:
            return
        
        self.send_instruction()

        
    
    def next_pos_callback(self, msg):
        self.next_pos = msg


    def send_instruction(self):
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("MoveGroup Action server ikke tilgjengelig.")
            return

        # Lag målposisjon
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = self.next_pos.data[0]
        pose.pose.position.y = self.next_pos.data[1]
        pose.pose.position.z = self.next_pos.data[2]
        pose.pose.orientation.w = 1.0  # nøytral orientering

        # Lag MoveGroup Goal (bare posisjonsbegrensning i dette enkle tilfellet)
        constraint = PositionConstraint()
        constraint.header.frame_id = self.frame_id
        constraint.link_name = self.link_name
        constraint.target_point_offset.x = 0.0
        constraint.target_point_offset.y = 0.0
        constraint.target_point_offset.z = 0.0

        shape = SolidPrimitive()
        shape.type = SolidPrimitive.BOX
        shape.dimensions = [0.01, 0.01, 0.01]  # liten kube rundt målet

        constraint.constraint_region.primitives.append(shape)
        constraint.constraint_region.primitive_poses.append(pose.pose)
        constraint.weight = 1.0

        constraints = Constraints()
        constraints.name = "target_pose"
        constraints.position_constraints.append(constraint)

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.group_name
        goal_msg.request.goal_constraints.append(constraints)
        goal_msg.request.max_velocity_scaling_factor = 0.3
        goal_msg.request.max_acceleration_scaling_factor = 0.3
        goal_msg.request.allowed_planning_time = 5.0

        self.get_logger().info(f"Sender mål til {self.next_pos}")

        future = self.client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)


    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Bevegelse fullført.")

        msg = Int32()
        msg.data = 2
        self.movement_state_pub.publish(msg)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Mål ble avvist.")
            return

        self.get_logger().info("Mål akseptert, venter på resultat...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

def main(args=None):
    rclpy.init(args=args)
    node = RobotCommNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()