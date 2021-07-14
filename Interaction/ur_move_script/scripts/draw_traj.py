import numpy as np
import math
import matplotlib as mpl
import matplotlib.pyplot as plt 



def main():
    file_path = "/home/xuchengjun/open_ros_codegen/data/ee_true.txt"

    data = []

    with open(file_path) as f:
        for line in f.readlines():
            line = line.strip(',')
            data.append(line)

    print(len(data))
    print(data[0][0])
    #plt.plot(data)
    # plt.plot(t, control, color='green',linewidth = 3)
    # plt.plot(t, pos, color='red',linewidth = 3)
    # plt.plot(t, vel, color='yellow',linewidth = 3)
    # plt.xlim((0.0, 1.0))
    # plt.ylim((-1.1, 11))
    #plt.show()

if __name__=="__main__":
    main()