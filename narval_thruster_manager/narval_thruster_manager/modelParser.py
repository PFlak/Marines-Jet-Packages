import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from tf2_msgs.msg import TFMessage


class ModelParser:
    def __init__(self, control_frame):
        self.control_frame = control_frame
        self.model = None
        self.tree = None

        self.frame_name = ''.join(c for c in self.control_frame if c.isalnum())

        self.rsp_node = rclpy.create_node(f"narval_thruster_manager_{self.frame_name}")
        self.client = self.rsp_node.create_client(Parameter, "robot_state_publisher")

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.req = Parameter
        

        


