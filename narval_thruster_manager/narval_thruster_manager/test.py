import numpy as np
import matplotlib.pyplot as plt


if __name__ == "__main__":
    path = '/home/dev/ros2_ws/src/jet/narval_thruster_manager/data/t200_20V_RPM.txt'
    with open(path,'r') as f:
        data = f.readlines()

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
    
    arr = np.array(arr)

    index1 = 0
    index2 = 0
    prev = 0

    first = arr[0][0]
    last = arr[len(arr) - 1][0]

    for i in range(1, len(arr)):
        prev = arr[i - 1][0]

        if prev != 0 and arr[i][0] == 0:
            index1 = i

        if prev == 0 and arr[i][0] != 0:
            index2 = i

    arr1 = arr[:index1]
    arr2 = arr[index2:]

    p1_fit = np.polyfit(arr[:index1, 0], arr[:index1, 1], 3)
    p2_fit = np.polyfit(arr[index2:, 0], arr[index2:, 1], 3)

    p1 = np.poly1d(p1_fit)
    p2 = np.poly1d(p2_fit)

    x = np.linspace(first, last, 100)

    y = np.where(x < 0, p1(x), np.where(x > 0, p2(x), 1500))

    plt.plot(x, y, '-', arr[:,0], arr[:,1], '.')
    plt.show()

        




    # polyfit = np.polyfit(arr[:,0], arr[:,1], 4)
    # p = np.poly1d(polyfit)
    # xp = np.linspace(-3, 3, 100)
    # _ = plt.plot(arr[:,0], arr[:,1], '.', xp, p(xp), '-')
    # plt.ylim(1100,1900)
    # plt.show()
