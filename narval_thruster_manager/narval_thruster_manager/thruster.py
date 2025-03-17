"""
Thruster Class
==============

The `Thruster` class represents a robotic thruster, allowing management of its transformation, force application, and wrench publishing.

Attributes
----------
_id : int
    Class-level variable used to assign unique IDs to thrusters.
"""
from typing import Literal

import rclpy
from rclpy.node import Node
from narval_thruster_manager.logger import Logger
from narval_thruster_manager.coefficient import Coefficient

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Transform
from std_msgs.msg import Float64
import math
import numpy as np
import tf_transformations as tf


ThrusterOutputType = Literal['SEP_WRENCH', 'SEP_FRC', 'SEP_PWM', 'SEP_RPM']

class Thruster:
    """
    Represents a thruster in a robotic system.

    :param msg: The transform of the thruster, including translation and rotation.
    :type msg: TransformStamped
    :param thruster_force_axis: The axis along which the thruster applies force ('x', 'y', or 'z'), defaults to 'x'.
    :type thruster_force_axis: str, optional
    :param publish_wrench: Whether to publish wrench messages for this thruster, defaults to True.
    :type publish_wrench: bool, optional
    :param pub_wrench_name: The base name of the wrench topic to publish, defaults to 'n_thruster_wrench'.
    :type pub_wrench_name: str, optional
    :param logger: Logger instance for logging messages. If None, a new logger is created, defaults to None.
    :type logger: Logger, optional
    :param node: ROS2 node for publishing and subscribing. If None, a new node is created, defaults to None.
    :type node: Node, optional
    """
        
    _id = 0

    def __init__(self,
                 msg: TransformStamped,
                 thruster_force_axis = "x",
                 publish_wrench_type: ThrusterOutputType = 'SEP_WRENCH',
                 pub_wrench_name: str = "n_thruster_wrench",
                 coefficient: Coefficient = None,
                 logger = None,
                 node = None):
        """
        Initializes the Thruster object with a unique ID, transformation data, force axis, and ROS2 publishers.
        """
        if logger:
            self.__logger = logger
        else:
            self.__logger = Logger("Thruster")

        self.publish_wrench_type = publish_wrench_type
        try:
            Thruster._id += 1
            self.__id = Thruster._id

            self.__do_publish_wrench = 'SEP' in publish_wrench_type.upper()

            if self.__do_publish_wrench:
                if node:
                    self.__node = node
                else:
                    self.__node = rclpy.create_node(f"thruster_node_{self.__id}")


                if publish_wrench_type == 'SEP_WRENCH':
                    self.__pub_wrench_name = f"{pub_wrench_name}_{self.__id}"
                    self.__pub_wrench = self.__node.create_publisher(WrenchStamped, self.__pub_wrench_name, 0)
                elif 'SEP' in publish_wrench_type:
                    self.__pub_wrench_name = f"{pub_wrench_name}_{self.__id}"
                    self.__pub_wrench = self.__node.create_publisher(Float64, self.__pub_wrench_name , 0)





            self.__force = .0

            if thruster_force_axis == "x":
                self.__thruster_force_axis = np.array([1, 0, 0])
            elif thruster_force_axis == "y":
                self.__thruster_force_axis = np.array([0, 1, 0])
            else:
                self.__thruster_force_axis = np.array([0, 0, 1])

            self.frame = msg.header.frame_id
            self.joint = msg.child_frame_id

            self.__trans = {
                "x": msg.transform.translation.x,
                "y": msg.transform.translation.y,
                "z": msg.transform.translation.z
            }

            self.__rot_quaternion = {
                "x": msg.transform.rotation.x,
                "y": msg.transform.rotation.y,
                "z": msg.transform.rotation.z,
                "w": msg.transform.rotation.w
            }

            self.__rot_euler = self.__quaternion_to_euler()
        except Exception as e:
            self.__logger.error(f"{e}")

        self.__logger.info(f"Thruster found: {self}")

        self._coefficient = coefficient
        

    def __quaternion_to_euler(self):
        
        q = self.__rot_quaternion
        __rot_euler = {}

        sinr_cosp = 2 * (q['w'] * q['x'] + q['y'] * q['z'])
        cosr_cosp = 1 - 2* (q['x'] * q["x"] + q['y'] * q['y'])
        __rot_euler['roll'] = math.atan2(sinr_cosp, cosr_cosp)

        sinp = math.sqrt(1 + 2 * (q['w'] * q['y'] - q['x'] *q['z']))
        cosp = math.sqrt(1 - 2 * (q['w'] * q['y'] - q['x'] *q['z']))
        __rot_euler['pitch'] = math.atan2(sinp, cosp) - math.pi / 2

        siny_cosp = 2 * (q['w'] * q['z'] + q['x'] * q['y'])
        cosy_cosp = 1 - 2 * (q['y'] * q['y'] + q['z'] * q['z'])
        __rot_euler['yaw'] = math.atan2(siny_cosp, cosy_cosp)

        return __rot_euler
    
    @property
    def translation_np_array(self):
        """
        Returns the translation vector of the thruster.

        :return: The translation vector as a NumPy array [x, y, z].
        :rtype: np.ndarray
        """

        return np.array([self.__trans['x'],
                         self.__trans['y'],
                         self.__trans['z']])
    
    @property
    def quaternion_rot_np_array(self):
        """
        Returns the rotation quaternion of the thruster.

        :return: The quaternion as a NumPy array [x, y, z, w].
        :rtype: np.ndarray
        """
        return np.array([self.__rot_quaternion['x'],
                         self.__rot_quaternion['y'],
                         self.__rot_quaternion['z'],
                         self.__rot_quaternion['w']])
    
    @property
    def euler_rot_np_array(self):
        """
        Returns the Euler angles of the thruster.

        :return: The Euler angles as a NumPy array [roll, pitch, yaw].
        :rtype: np.ndarray
        """
        return np.array([self.__rot_euler['roll'],
                         self.__rot_euler['pitch'],
                         self.__rot_euler['yaw']])
    
    @property
    def quaternion_rotation_matrix(self):
        """
        Calculates and returns the quaternion rotation matrix.

        :return: The 3x3 rotation matrix.
        :rtype: np.ndarray
        """
        # Extract the values
        q0 = self.__rot_quaternion['w']
        q1 = self.__rot_quaternion['x']
        q2 = self.__rot_quaternion['y']
        q3 = self.__rot_quaternion['z']
        
        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)
        
        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)
        
        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1
        
        # 3x3 rotation matrix
        rot_matrix = np.array([[r00, r01, r02],
                            [r10, r11, r12],
                            [r20, r21, r22]])
                                
        return rot_matrix
    
    def update_force(self, f):
        """
        Updates the force applied by the thruster.

        :param f: The new force value to apply.
        :type f: float
        """
        self.__force = f
        if self.__do_publish_wrench and self.__node:
            self.__publish_wrench()

    @property
    def force(self):
        """
        Returns the current force applied by the thruster.

        :return: The current force value.
        :rtype: float
        """

        return self.__force
    

    @property
    def wrench_msg(self):
        """
        Creates a WrenchStamped message for the thruster.

        :return: The wrench message with force and torque data.
        :rtype: WrenchStamped
        """

        wrench = WrenchStamped()
        now = self.__node.get_clock().now()

        wrench.header.frame_id = self.joint
        wrench.header.stamp = now.to_msg()

        if self.__thruster_force_axis[0]:
            wrench.wrench.force.x = -self.__force
        elif self.__thruster_force_axis[1]:
            wrench.wrench.force.y = -self.__force
        else:      
            wrench.wrench.force.z = -self.__force

        wrench.wrench.torque.x = .0
        wrench.wrench.torque.y = .0
        wrench.wrench.torque.z = .0

        return wrench

    @property
    def node(self) -> Node:
        """
        Returns the ROS2 node associated with the thruster.

        :return: The ROS2 node.
        :rtype: Node
        """
                
        return self.__node

    @property
    def thruster_force_axis(self):
        """
        Returns the thruster force axis in the world frame.

        :return: The force axis vector as a NumPy array [x, y, z].
        :rtype: np.ndarray
        """

        q = (self.__rot_quaternion['x'],
             self.__rot_quaternion['y'],
             self.__rot_quaternion['z'], 
             self.__rot_quaternion['w'])
        return np.array(self.__qv_mult(q, self.__thruster_force_axis))

    def __publish_wrench(self):
        wrench = self.wrench_msg

        if self.publish_wrench_type == 'SEP_WRENCH':
            self.__pub_wrench.publish(wrench)
        elif self.publish_wrench_type == 'SEP_FRC': 
            force_msg = Float64()
            force_msg.data = self.__force
            self.__pub_wrench.publish(force_msg)
        elif self.publish_wrench_type == 'SEP_RPM': 
            rpm_msg = Float64()
            rpm_msg.data = self._coefficient.calc('RPM', self.__force)
            self.__pub_wrench.publish(rpm_msg)
        elif self.publish_wrench_type == 'SEP_PWM':
            pwm_msg = Float64()
            pwm_msg.data = self._coefficient.calc('PWM', self.__force)
            self.__pub_wrench.publish(pwm_msg)

    @staticmethod
    def __qv_mult(q1, v1):
        # comment this out if v1 doesn't need to be a unit vector
        v1 = tf.unit_vector(v1)
        q2 = list(v1)
        q2.append(0.0)
        return tf.quaternion_multiply(
            tf.quaternion_multiply(q1, q2), 
            tf.quaternion_conjugate(q1)
        )[:3]

    def __str__(self):
        return f"Joint: {self.joint}; Frame: {self.frame}; Translation: {str(self.__trans)}"