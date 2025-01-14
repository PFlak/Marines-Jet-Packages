from narval_thruster_manager.thruster import Thruster
from narval_thruster_manager.logger import Logger
from geometry_msgs.msg import WrenchStamped

import numpy as np

class ThrusterAllocationMatrix:
    def __init__(self,_thruster_list: list[Thruster] = []):
        self.__thruster_list = _thruster_list
        self.__tam = None

        self.__logger = Logger(self.__module__)

    def add_thruster(self, thruster: Thruster):
        try:
            self.__thruster_list.append(thruster)
            
        except Exception as e:
            self.__logger.error(f"Could not add thruster: {str(e)}")
            return
        
        self.__logger.info(f"Thruster added: {str(thruster)}")

    def add_thrusters(self, thruster_list: list[Thruster]):
        i = 0
        j = len(thruster_list)
        for thruster in thruster_list:
            try:
                self.__thruster_list.append(thruster)
            
            except Exception as e:
                self.__logger.error(f"Could not add thruster: {str(e)}")
                continue
            
            self.__logger.info(f"Thruster added: {str(thruster)}")
        
        self.__logger.info(f"{i} / {j} Thrusters added")

    def get_thrusters(self) -> list[Thruster]: 
        return self.__thruster_list


    def calculate_TAM(self):
        T = []
        
        for thruster in self.__thruster_list:
            try:
                rotation_matrix = thruster.get_quaternion_rotation_matrix()
                direction = rotation_matrix @ thruster.get_thruster_force_axis()

                col = np.zeros(6)
                col[:3] = direction
                col[3:] = np.cross(rotation_matrix, direction)
                T.append(col)

            except Exception as e:
                self.__logger.error(f"Error during calculating TAM: {str(e)}")

        self.__tam = np.array(T).T

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

        for i, thruster in enumerate(self.__thruster_list):
            thruster.update_force(thruster_outputs[i])

        return thruster_outputs, self.get_thrusters()