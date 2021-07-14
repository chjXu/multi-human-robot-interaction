import numpy as np
import math
import matplotlib as mpl
import matplotlib.pyplot as plt 


pos = [0.000000, 0.000000, 0.000000, 0.405000,1.805000,4.205000, 7.605000,10.000000, 10.000000]
vel = [0.000000,0.000000,0.000000,0.900000,1.900000,2.900000,3.900000,1.000000,1.000000]
control = [1, 1, 1, 1, 0.959637, 0.58812, 0.309033, 0.119533, 0.0171906]
t = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]

init_state = [0, 0]
ref_state = [10, 1]
sample_time = 0.1

all_state = []

def update(state, control):
    return [state[1] * sample_time + 0.5 * control * sample_time * sample_time, 
                    control * sample_time]

def main():
    plt.plot(t, control, color='green',linewidth = 3)
    plt.plot(t, pos, color='red',linewidth = 3)
    plt.plot(t, vel, color='yellow',linewidth = 3)
    plt.xlim((0.0, 1.0))
    plt.ylim((-1.1, 11))
    plt.show()

if __name__=="__main__":
    main()
        