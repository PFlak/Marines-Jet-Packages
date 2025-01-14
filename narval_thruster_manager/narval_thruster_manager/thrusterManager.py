import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile
from rclpy.logging import LoggingSeverity

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, WrenchStamped
from narval_thruster_manager.thruster import Thruster
from narval_thruster_manager.thrusterAllocationMatrix import ThrusterAllocationMatrix

from narval_thruster_manager.logger import Logger

class ThrusterManager:
    def __init__(self, node: Node):
        
        self.__node = node
        self.__logger = self.__node.get_logger().set_level(LoggingSeverity.INFO)

        self.__node.declare_parameter("thruster_prefix", "t200")
        self.__node.declare_parameter("thruster_force_axis", "z")
        self.__node.declare_parameter("publish_thruster_wrench", True)
        self.__node.declare_parameter("thruster_wrench_publisher_name", "n_thruster_wrench")

        self.__node.declare_parameter("sub_input_wrench_topic_name", "joy_wrench_stmp")

        self.param_thruster_prefix = self.__node.get_parameter("thruster_prefix").value
        self.param_thruster_force_axis = self.__node.get_parameter("thruster_force_axis").value
        self.param_publish_thruster_wrench = self.__node.get_parameter("publish_thruster_wrench").value
        self.param_thruster_wrench_publisher_name = self.__node.get_parameter("thruster_wrench_publisher_name").value

        self.param_sub_input_wrench_topic_name = self.__node.get_parameter("sub_input_wrench_topic_name").value

        self.sub_tf_static = self.__node.create_subscription(TFMessage, "tf_static", self.cb_tf_static, 10)
        self.sub_input_wrench = self.__node.create_subscription(WrenchStamped, self.param_sub_input_wrench_topic_name, self.cb_input_wrench, 0)


        self.thrusters = []

        self.TAMManager = ThrusterAllocationMatrix(node=self.__node, logger=self.__logger)

    def cb_tf_static(self, msg: TFMessage):
        transforms = msg.transforms

        error_rate = 0

        for i in range(len(transforms)):
            try:
                transform = transforms[i]

                if self.param_thruster_prefix in transform.child_frame_id:

                    thruster = Thruster(msg=transform,
                                        logger=self.__node.get_logger(),
                                        node=self.__node,
                                        publish_wrench=self.param_publish_thruster_wrench,
                                        pub_wrench_name=self.param_thruster_wrench_publisher_name,
                                        thruster_force_axis=self.param_thruster_force_axis)
                    
                    self.thrusters.append(thruster)
                    check = self.TAMManager.add_thruster(thruster)

                    if not check:
                        raise Exception
                    
                    
            except Exception as e:
                self.__logger.error(f"Could not add thruster: {str(e)}")
                error_rate += 1
        
        if len(self.TAMManager.get_thrusters()) == 0:
            self.__logger.warning("Did not found any thrusters")
            return
        elif len(self.TAMManager.get_thrusters()) == 6:
            self.TAMManager.calculate_TAM()
        else:
            self.__logger.info(f"Found: {len(self.TAMManager.get_thrusters())} Thrusters")
            


    def cb_input_wrench(self, msg: WrenchStamped):
        if len(self.TAMManager.get_thrusters()) < 6:
            return
        self.TAMManager.solve_wrench(msg)