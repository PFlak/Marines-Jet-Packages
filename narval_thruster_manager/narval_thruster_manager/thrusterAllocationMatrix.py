import rclpy
from rclpy.node import Node

from narval_thruster_manager.thruster import Thruster
from narval_thruster_manager.logger import Logger
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import TransformStamped

import numpy as np

class ThrusterAllocationMatrix:
    def __init__(self, _thruster_list = None, logger = None, node = None):
        if _thruster_list is None:
            self._thruster_list: list[Thruster] = []
        else:
            self._thruster_list = _thruster_list
        self.__tam = None
        if logger:
            self.__logger = logger
        else:
            self.__logger = Logger("ThrusterAllocationMatrix")

    def add_thruster(self, thruster):
        try:
            self._thruster_list.append(thruster)
            
        except Exception as e:
            self.__logger.error(f"Could not add thruster: {e}")
            return False
        
        self.__logger.info(f"Thruster added: {thruster.joint} to TAM")
        return True
        

    def add_thrusters(self, thruster_list):
        i = 0
        j = len(thruster_list)
        for thruster in thruster_list:
            try:
                self._thruster_list.append(thruster)
            
            except Exception as e:
                self.__logger.error(f"Could not add thruster: {str(e)}")
                continue
            
            self.__logger.info(f"Thruster added: {str(thruster.joint)} to TAM")
        
        self.__logger.info(f"{i} / {j} Thrusters added")

    def get_thrusters(self) -> list[Thruster]: 
        return self._thruster_list


    def calculate_TAM(self):
        T = []
        
        for thruster in self._thruster_list:
            try:
                f = thruster.get_thruster_force_axis()
                r = thruster.get_translation_np_array()

                self.__logger.info(f"F:\n{f}\nR:\n{r}")

                col = np.zeros(6)
                col[:3] = f
                col[3:] = np.cross(r, f)
                T.append(col)

            except Exception as e:
                self.__logger.error(f"Error during calculating TAM: {str(e)}")

        self.__tam = np.array(T).T
        self.__logger.info(f"{self.__tam}")

    def get_TAM(self):

        if not self.__tam:
            self.calculate_TAM()

        return self.__tam
    
    def solve_wrench(self, wrench: WrenchStamped):
        desired_wrench = np.array([wrench.wrench.force.x,
                                   wrench.wrench.force.y,
                                   wrench.wrench.force.z,
                                   wrench.wrench.torque.x,
                                   wrench.wrench.torque.y,
                                   wrench.wrench.torque.z])
        
        tam_pseudoinverse = np.linalg.pinv(self.__tam)

        thruster_outputs = np.dot(tam_pseudoinverse, desired_wrench)

        for i, thruster in enumerate(self._thruster_list):
            thruster.update_force(thruster_outputs[i])

        return thruster_outputs, self.get_thrusters()