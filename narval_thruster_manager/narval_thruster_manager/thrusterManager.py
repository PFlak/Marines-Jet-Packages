"""
Thruster Manager
================

The `ThrusterManager` class manages thrusters in a robotic system, subscribes to transformation data, and 
allocates forces and torques to the thrusters using a `ThrusterAllocationMatrix`. It interfaces with ROS2 nodes
for parameter configuration, subscriptions, and logging.

Attributes
----------
__node : Node
    The ROS2 node associated with the thruster manager.
__logger : Logger
    Logger instance for logging thruster manager events and errors.
thrusters : list[Thruster]
    List of thrusters managed by the Thruster Manager.
TAMManager : ThrusterAllocationMatrix
    Manages the Thruster Allocation Matrix (TAM) for force and torque distribution.
param_thruster_prefix : str
    Prefix used to identify thruster transforms.
param_thruster_force_axis : str
    Axis along which the thruster generates force (e.g., 'x', 'y', or 'z').
param_publish_thruster_wrench : bool
    Whether to publish the wrench (force and torque) for each thruster.
param_thruster_wrench_publisher_name : str
    Name of the topic where thruster wrench messages are published.
param_sub_input_wrench_topic_name : str
    Name of the topic where input wrench messages are subscribed.
sub_tf_static : Subscription
    Subscription to the "tf_static" topic for thruster transform data.
sub_input_wrench : Subscription
    Subscription to the input wrench topic.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile
from rclpy.logging import LoggingSeverity

from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TransformStamped, WrenchStamped
from narval_thruster_manager.thruster import Thruster
from narval_thruster_manager.thrusterAllocationMatrix import ThrusterAllocationMatrix
from narval_thruster_manager.coefficient import Coefficient

from narval_thruster_manager.logger import Logger

class ThrusterManager:
    """
    Handles thruster discovery, management, and force allocation using ROS2.

    :param node: The ROS2 node that initializes the ThrusterManager.
    :type node: Node
    """

    def __init__(self, node: Node):
        """
        Initializes the ThrusterManager, declares parameters, and sets up subscriptions.
        """

        self.__node = node
        self.__logger = self.__node.get_logger().set_level(LoggingSeverity.INFO)

        self.__node.declare_parameter("thruster_prefix", "t200")
        self.__node.declare_parameter("thruster_force_axis", "z")
        self.__node.declare_parameter("thruster_wrench_publisher_name", "n_thruster_wrench")
        self.__node.declare_parameter("output_type", 'SEP_PWM')
        self.__node.declare_parameter("thruster_voltage", 12)

        self.__node.declare_parameter("sub_input_wrench_topic_name", "joy_wrench_stmp")

        self.param_thruster_prefix = self.__node.get_parameter("thruster_prefix").value
        self.param_thruster_force_axis = self.__node.get_parameter("thruster_force_axis").value
        self.param_thruster_wrench_publisher_name = self.__node.get_parameter("thruster_wrench_publisher_name").value

        self.param_output_type = self.__node.get_parameter('output_type').value
        self.param_thruster_voltage = self.__node.get_parameter('thruster_voltage').value

        self.param_sub_input_wrench_topic_name = self.__node.get_parameter("sub_input_wrench_topic_name").value

        self.sub_tf_static = self.__node.create_subscription(TFMessage, "tf_static", self.cb_tf_static, 10)
        self.sub_input_wrench = self.__node.create_subscription(WrenchStamped, self.param_sub_input_wrench_topic_name, self.cb_input_wrench, 0)

        if 'SEP' not in self.param_output_type:
            self.pub_output = self.__node.create_publisher(Float64MultiArray, f"/thrusters/{self.param_output_type}",0)

        self.coefficient = Coefficient(prefix=self.param_thruster_prefix,
                                       power=self.param_thruster_voltage)

        self.thrusters = []

        self.TAMManager = ThrusterAllocationMatrix(node=self.__node, logger=self.__logger)

    def cb_tf_static(self, msg: TFMessage):
        """
        Callback for the "tf_static" topic, responsible for discovering thrusters.

        Parses the static transforms and adds thrusters whose child frame ID matches the specified prefix.
        Also initializes the Thruster Allocation Matrix (TAM) if enough thrusters are found.

        :param msg: Message containing static transforms.
        :type msg: TFMessage
        """

        transforms = msg.transforms

        error_rate = 0

        for i in range(len(transforms)):
            try:
                transform = transforms[i]

                if self.param_thruster_prefix in transform.child_frame_id:

                    thruster = Thruster(msg=transform,
                                        logger=self.__node.get_logger(),
                                        node=self.__node,
                                        pub_wrench_name=self.param_thruster_wrench_publisher_name,
                                        thruster_force_axis=self.param_thruster_force_axis,
                                        coefficient=self.coefficient,
                                        publish_wrench_type=self.param_output_type
                                        )
                    
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
        elif len(self.TAMManager.get_thrusters()) == 8:
            self.TAMManager.calculate_TAM()
        else:
            pass
            # self.__logger.info(f"Found: {len(self.TAMManager.get_thrusters())} Thrusters")
            


    def cb_input_wrench(self, msg: WrenchStamped):
        """
        Callback for the input wrench topic, responsible for solving the Thruster Allocation Matrix (TAM).

        Allocates forces and torques to the thrusters based on the desired wrench. If fewer than six thrusters
        are configured, this method does nothing.

        :param msg: Desired wrench (forces and torques) for the thrusters to achieve.
        :type msg: WrenchStamped
        """

        if len(self.TAMManager.get_thrusters()) < 6:
            return
        
        thruster_forces = self.TAMManager.solve_wrench(msg)[0]

        try:
            if self.pub_output:
                output: Float64MultiArray = Float64MultiArray()
                
                if self.param_output_type == 'PWM':
                    for i, thruster in enumerate(self.TAMManager.get_thrusters()):
                        output.data.append(thruster._coefficient.calc('PWM', thruster.force))
                elif self.param_output_type == 'RPM':
                    for i, thruster in enumerate(self.TAMManager.get_thrusters()):
                        output.data.append(thruster._coefficient.calc('RPM', thruster.force))
                elif self.param_output_type == 'STONEFISH':
                    for i, thruster in enumerate(self.TAMManager.get_thrusters()):
                        output.data.append(thruster._coefficient.calc('STONEFISH', thruster.force))
                elif self.param_output_type == 'FRC':
                    [output.data.append(x) for x in thruster_forces]

                self.pub_output.publish(output)
        except:
            pass

