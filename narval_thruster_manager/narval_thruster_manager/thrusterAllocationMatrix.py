from narval_thruster_manager.thruster import Thruster
from narval_thruster_manager.logger import Logger

class ThrusterAllocationMatrix:
    def __init__(self,_thruster_list: list[Thruster] = []):
        self.__thruster_list = _thruster_list

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

    