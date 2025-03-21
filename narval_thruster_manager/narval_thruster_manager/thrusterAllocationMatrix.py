"""
Thruster Allocation Matrix (TAM)
=================================

The `ThrusterAllocationMatrix` class manages a list of thrusters and computes the Thruster Allocation Matrix (TAM),
which is used to allocate forces and torques to individual thrusters based on a desired wrench.

Attributes
----------
_thruster_list : list[Thruster]
    List of thrusters managed by the TAM.
__tam : np.ndarray
    The Thruster Allocation Matrix, a 6xN matrix (6 for forces and torques, N for the number of thrusters).
__logger : Logger
    Logger instance for logging TAM-related events and errors.
"""

import rclpy
from rclpy.node import Node

from narval_thruster_manager.thruster import Thruster
from narval_thruster_manager.logger import Logger
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64MultiArray
import scipy.sparse as sp
import osqp
import numpy as np

class ThrusterAllocationMatrix:
    """
    Handles thruster management and computation of the Thruster Allocation Matrix (TAM).

    :param _thruster_list: Initial list of thrusters to include, defaults to an empty list.
    :type _thruster_list: list[Thruster], optional
    :param logger: Logger instance for logging messages. If None, a default logger is created, defaults to None.
    :type logger: Logger, optional
    :param node: ROS2 node for managing thrusters. If None, thrusters must manage their own nodes, defaults to None.
    :type node: Node, optional
    """
        
    def __init__(self, _thruster_list = None, logger = None, node = None):
        """
        Initializes the ThrusterAllocationMatrix with optional thrusters, logger, and ROS2 node.
        """

        if _thruster_list is None:
            self._thruster_list: list[Thruster] = []
        else:
            self._thruster_list = _thruster_list
        self.__tam = None
        if logger:
            self.__logger = logger
        else:
            self.__logger = Logger("ThrusterAllocationMatrix")

        self.num_thrusters = 0
        self.osqp_solver = None

    def add_thruster(self, thruster):
        """
        Adds a single thruster to the TAM.

        :param thruster: The thruster to add.
        :type thruster: Thruster
        :return: True if the thruster was successfully added, False otherwise.
        :rtype: bool
        """

        try:
            self._thruster_list.append(thruster)
            self.num_thrusters += 1
            
        except Exception as e:
            self.__logger.error(f"Could not add thruster: {e}")
            return False
        
        self.__logger.info(f"Thruster added: {thruster.joint} to TAM")
        return True
        

    def add_thrusters(self, thruster_list):
        """
        Adds multiple thrusters to the TAM.

        :param thruster_list: A list of thrusters to add.
        :type thruster_list: list[Thruster]
        """

        i = 0
        j = len(thruster_list)
        for thruster in thruster_list:
            try:
                self.add_thruster(thruster)
            
            except Exception as e:
                continue
        
        self.__logger.info(f"{i} / {j} Thrusters added")

    def get_thrusters(self) -> list[Thruster]: 
        """
        Returns the list of thrusters managed by the TAM.

        :return: The list of thrusters.
        :rtype: list[Thruster]
        """

        return self._thruster_list

    def calculate_TAM(self):
        """
        Calculates the Thruster Allocation Matrix (TAM) based on the current thrusters' positions and force axes.

        The TAM is a 6xN matrix where:
        - The first 3 rows correspond to forces (x, y, z).
        - The last 3 rows correspond to torques (roll, pitch, yaw).
        """

        T = []
        
        for thruster in self._thruster_list:
            try:
                f = thruster.thruster_force_axis
                r = thruster.translation_np_array

                self.__logger.info(f"F:\n{f}\nR:\n{r}")

                col = np.zeros(6)
                col[:3] = f
                col[3:] = np.cross(r, f)
                T.append(col)

            except Exception as e:
                self.__logger.error(f"Error during calculating TAM: {str(e)}")

        self.num_thrusters = len(self._thruster_list)
        self.__tam = np.array(T).T
        self.__logger.info(f"{self.__tam}")

        self.osqp_solver = self.setup_osqp()

    def get_TAM(self):
        """
        Retrieves the Thruster Allocation Matrix (TAM). If the TAM is not calculated, it will calculate it.

        :return: The Thruster Allocation Matrix.
        :rtype: np.ndarray
        """

        if not self.__tam:
            self.calculate_TAM()

        return self.__tam
    
    def solve_wrench(self, wrench: WrenchStamped, algo = 'pseudoinv'):
        """
        Solves for the required thruster forces to achieve a desired wrench.

        :param wrench: The desired wrench (forces and torques) to achieve.
        :type wrench: WrenchStamped
        :return: A tuple containing:
            - An array of thruster forces.
            - The list of thrusters.
        :rtype: tuple[np.ndarray, list[Thruster]]
        """

        desired_wrench = np.array([wrench.wrench.force.x,
                                   wrench.wrench.force.y,
                                   wrench.wrench.force.z,
                                   wrench.wrench.torque.x,
                                   wrench.wrench.torque.y,
                                   wrench.wrench.torque.z])
        
        if algo == 'pseudoinv':
            tam_pseudoinverse = np.linalg.pinv(self.__tam)

            thruster_outputs = np.dot(tam_pseudoinverse, desired_wrench)
        
        elif algo == 'qp':
            self.osqp_solver.update(l=desired_wrench, u=desired_wrench)

            result = self.osqp_solver.solve()

            if result.info.status_val in [osqp.constant('OSQP_SOLVED')]:
                thruster_outputs = result.x

            else:
                self.__logger.warning("QP solver failed, returning zero forces.")
                thruster_outputs = np.zeros(self.num_thrusters)
        
        for i, thruster in enumerate(self._thruster_list):
            thruster.update_force(thruster_outputs[i])

        thruster_outputs = self.scale(thruster_outputs)

        return thruster_outputs, self.get_thrusters()
    
    def setup_osqp(self):
        
        P = sp.eye(self.num_thrusters).tocsc()

        A = sp.csc_matrix(self.__tam)

        l = np.full(6, 5)
        u = np.full(6, 10)

        solver = osqp.OSQP()
        solver.setup(P, np.zeros(self.num_thrusters), A, l, u, verbose=False)

        return solver
    
    def scale(self, thruster_outputs):

        thrust_coef = 0.167
        torque_coef = 0.016

        rpm_output = np.sign(thruster_outputs) * np.sqrt(np.abs(thruster_outputs) / thrust_coef)

        return rpm_output
    
    
