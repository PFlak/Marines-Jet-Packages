import rclpy
from rclpy.node import Node
from narval_thruster_manager.logger import Logger

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import WrenchStamped

import math
import numpy as np
from typing import Literal
from enum import Enum

ThrusterForceAxis = Literal['x', 'y', 'z']

class Thruster:
    _id = 0
    def __init__(self, msg: TransformStamped, thruster_force_axis: ThrusterForceAxis = 'x' ,publish_wrench: bool = True, pub_wrench_name: str = "n_thruster_wrench"):

        Thruster._id += 1
        self.__id = Thruster._id

        self.__do_publish_wrench = publish_wrench

        if self.__do_publish_wrench:
            self.__node = rclpy.create_node(f"n_thruster_{self.__id}")

            self.__pub_wrench_name = f"{pub_wrench_name}_{self.__id}"

            self.__pub_wrench = self.__node.create_publisher(WrenchStamped, self.__pub_wrench_name, 0)

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
    
    def get_translation_np_array(self):
        return np.array([self.__trans['x'],
                         self.__trans['y'],
                         self.__trans['z']])
    
    def get_quaternion_rot_np_array(self):
        return np.array([self.__rot_quaternion['x'],
                         self.__rot_quaternion['y'],
                         self.__rot_quaternion['z'],
                         self.__rot_quaternion['w']])
    
    def get_euler_rot_np_array(self):
        return np.array([self.__rot_euler['roll'],
                         self.__rot_euler['pitch'],
                         self.__rot_euler['yaw']])
    
    def get_quaternion_rotation_matrix(self):

        # Extract the values
        q0 = self.__rot_quaternion['x']
        q1 = self.__rot_quaternion['y']
        q2 = self.__rot_quaternion['z']
        q3 = self.__rot_quaternion['w']
        
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
        self.__force = f
        if self.__do_publish_wrench and self.__node:
            self.__publish_wrench()

    def get_force(self):
        return self.__force

    def get_wrench_msg(self):
        wrench = WrenchStamped()
        now = self.__node.get_clock().now()

        wrench.header.frame_id = self.joint
        wrench.header.stamp = now

        wrench.wrench.force.x = self.__force
        wrench.wrench.force.y = .0
        wrench.wrench.force.z = .0

        wrench.wrench.torque.x = .0
        wrench.wrench.torque.y = .0
        wrench.wrench.torque.z = .0

        return wrench

    def get_node(self) -> Node:
        return self.__node

    def get_thruster_force_axis(self):
        return self.__thruster_force_axis

    def __publish_wrench(self):
        wrench = self.get_wrench_msg()
        self.__pub_wrench.publish(wrench)

    def __str__(self):
        return f"Joint: {self.joint}; Frame: {self.frame}; Translation: {str(self.__trans)}"