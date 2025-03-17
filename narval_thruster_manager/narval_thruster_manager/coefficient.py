from typing import Literal
from dataclasses import dataclass
import numpy as np
import matplotlib.pyplot as plt
import os
from ament_index_python.packages import get_package_share_directory

ThrusterPowerMode = Literal[10, 12, 14, 16, 18, 20]

@dataclass
class CoefFunc:
    func: np.poly1d
    minimum: float | int
    maximum: float | int

    over_value: float | int
    below_value: float | int
    
    @property
    def range(self):
        return (self.minimum, self.maximum)
    
    def in_range(self, val: float | int) -> int:
        if np.all(val > self.minimum) and np.all(val < self.maximum):
            return 1
        elif np.all(val <= self.minimum):
            return -1
        else:
            return 0
    
    @staticmethod
    def from_dict(data: dict) -> 'CoefFunc':
        return CoefFunc(**data)
    
@dataclass
class CoefFuncArray:
    func_arr: list[CoefFunc]

    @property
    def range(self):
        return (self.func_arr[0].minimum, self.func_arr[-1].maximum)

    @staticmethod
    def from_dict(data: dict) -> 'CoefFuncArray':
        return CoefFuncArray(**data).sort()
    
    def calc(self, value: int | float):
        prev = None

        for func in self.func_arr:
            
            is_in_range = func.in_range(val=value)

            if is_in_range == 1:
                return func.func(val=value)
            
            elif is_in_range == -1:
                if prev is None or prev == 0:
                    return func.below_value

        return self.func_arr[-1].over_value

    def sort(self):
        self.func_arr.sort(key= lambda x: x.minimum)



class Coefficient:
    def __init__(self, prefix: str = 't200',
                 power: ThrusterPowerMode = 12,
                 max_rpm: int = 3931,
                 max_frc: float = 6.72,
                 min_frc: float  = -5.04
                 ):
        package_path = get_package_share_directory('narval_thruster_manager')

        pwm_file_path = f"{prefix}_{power}V_PWM.txt"
        rpm_file_path = f"{prefix}_{power}V_RPM.txt"

        pwm_file_path = os.path.join(package_path, 'data' ,pwm_file_path)
        rpm_file_path = os.path.join(package_path, 'data' ,rpm_file_path)

        # if os.path.exists(pwm_file_path):
        #     raise FileNotFoundError(f"{pwm_file_path} does not exist.")
        
        # if os.path.exists(rpm_file_path):
        #     raise FileNotFoundError(f"{rpm_file_path} does not exist.")
        
        self.pwm_data = self.prepare_data(pwm_file_path)
        self.rpm_data = self.prepare_data(rpm_file_path)

        self.max_rpm = max_rpm
    
        self.max_frc = max_frc
        self.min_frc = min_frc

        self.update()
        
    @staticmethod    
    def prepare_data(path):
        '''
        Prepares data from file.
        '''

        try:
            with open(path, 'r') as f:
                data = f.readlines()
        except Exception as e:
            raise e

        try:
            arr = []
            for line in data:
                sep_line = line.split('\t')
                try:
                    while True:
                        sep_line.remove('')
                except:
                    pass
                finally:
                    sep_line = (float(sep_line[0]), int(sep_line[1]))
                    arr.append(sep_line)
            
            return np.array(arr)
        
        except Exception as e:
            raise e

    def set_pwm_coef(self, pwm_data:np.ndarray):
        self.pwm_data = pwm_data

    def set_rpm_coef(self, rpm_data:np.ndarray):
        self.rpm_data = rpm_data

    def update(self):
        self._calc_pwm_attr()
        self._calc_rpm_attr()
        self._calc_stonefish_attr()

    def _calc_pwm_attr(self):
        index1 = 0
        index2 = 0
        prev = 0

        arr = self.pwm_data.copy()

        first = arr[0][0]
        below_value = arr[0][1]

        last = arr[-1][0]
        over_value = arr[-1][1]

        for i in range(1, len(arr)):
            prev = arr[i - 1][0]

            if prev != 0 and arr[i][0] == 0:
                index1 = i

            if prev == 0 and arr[i][0] != 0:
                index2 = i
        
        p1_fit = np.polyfit(arr[:index1, 0], arr[:index1, 1], 3)
        p2_fit = np.polyfit(arr[index2:, 0], arr[index2:, 1], 3)

        p1 = np.poly1d(p1_fit)
        p2 = np.poly1d(p2_fit)

        coef_p1 = CoefFunc(
            func=p1,
            minimum=first,
            maximum=arr[index1 + 1][0],
            below_value=below_value,
            over_value=1550
        )

        coef_p2 = CoefFunc(
            func=p2,
            minimum=arr[index2 - 1][0],
            maximum=last,
            below_value=1550,
            over_value=over_value
        )

        self.pwm_coef_func = CoefFuncArray(
            [coef_p1, coef_p2]
        )
        
        self.pwm_coef_func.sort()

    def _calc_rpm_attr(self):
        index1 = 0
        index2 = 0
        prev = 0

        arr = self.rpm_data.copy()

        for i in range(1, len(arr)):
            prev = arr[i - 1][0]

            if prev != 0 and arr[i][0] == 0:
                index1 = i

            if prev == 0 and arr[i][0] != 0:
                index2 = i
        
        arr[:index1, 1] = -arr[:index1, 1]

        first = arr[0][0]
        below_value = arr[0][1]

        last = arr[-1][0]
        over_value = arr[-1][1]

        p1_fit = np.polyfit(arr[:index1, 0], arr[:index1, 1], 3)
        p2_fit = np.polyfit(arr[index2:, 0], arr[index2:, 1], 3)

        p1 = np.poly1d(p1_fit)
        p2 = np.poly1d(p2_fit)

        coef_p1 = CoefFunc(
            func=p1,
            minimum=first,
            maximum=arr[index1][0],
            below_value=below_value,
            over_value=0
        )

        coef_p2 = CoefFunc(
            func=p2,
            minimum=arr[index2 - 1][0],
            maximum=last,
            below_value=0,
            over_value=over_value
        )

        self.rpm_coef_func = CoefFuncArray(
            [coef_p1, coef_p2]
        )

        self.rpm_coef_func.sort()

    def _calc_stonefish_attr(self):
        index1 = 0
        index2 = 0
        prev = 0

        arr = self.rpm_data.copy()

        for i in range(1, len(arr)):
            prev = arr[i - 1][0]

            if prev != 0 and arr[i][0] == 0:
                index1 = i

            if prev == 0 and arr[i][0] != 0:
                index2 = i
        
        arr[:index1, 1] = -arr[:index1, 1]

        arr[:,1] = arr[:,1] / self.max_rpm

        first = arr[0][0]
        below_value = arr[0][1]

        last = arr[-1][0]
        over_value = arr[-1][1]

        p1_fit = np.polyfit(arr[:index1, 0], arr[:index1, 1], 3)
        p2_fit = np.polyfit(arr[index2:, 0], arr[index2:, 1], 3)

        p1 = np.poly1d(p1_fit)
        p2 = np.poly1d(p2_fit)

        coef_p1 = CoefFunc(
            func=p1,
            minimum=first,
            maximum=arr[index1][0],
            below_value=below_value,
            over_value=0
        )

        coef_p2 = CoefFunc(
            func=p2,
            minimum=arr[index2 - 1][0],
            maximum=last,
            below_value=0,
            over_value=over_value
        )

        self.stonefish_coef_func = CoefFuncArray(
            [coef_p1, coef_p2]
        )

        self.stonefish_coef_func.sort()

        

    def visualize_pwm(self):
        range_min, range_max = self.pwm_coef_func.range
        x = np.linspace(range_min, range_max, 100)
        y = [self.pwm_coef_func.calc(val) for val in x]

        fig = plt.plot(x, y)
        plt.xlabel('Thrust [kg f]')
        plt.ylabel('PWM')
        plt.show()

    def visualize_rpm(self):
        range_min, range_max = self.rpm_coef_func.range
        x = np.linspace(range_min, range_max, 100)
        y = [self.rpm_coef_func.calc(val) for val in x]
        fig =plt.plot(x, y)
        plt.xlabel('Thrust [kg f]')
        plt.ylabel('RPM')
        plt.show()

    def visualize_stonefish(self):
        range_min, range_max = self.min_frc, self.max_frc
        x = np.linspace(range_min, range_max, 100)
        y = [self.stonefish_coef_func.calc(val) for val in x]
        fig =plt.plot(x, y)
        plt.xlabel('Thrust [kg f]')
        plt.ylabel('Stonefish')
        plt.show()

    def calc(self, type: Literal['PWM', 'RPM', 'STONEFISH'], value: int | float):
        if type == 'PWM':
            return self.pwm_coef_func.calc(value=value)
        elif type == 'RPM':
            return self.rpm_coef_func.calc(value=value)
        elif type == 'STONEFISH':
            return self.stonefish_coef_func.calc(value=value)
        

if __name__ == '__main__':
    coef = Coefficient(power=12)
    coef.visualize_stonefish()
