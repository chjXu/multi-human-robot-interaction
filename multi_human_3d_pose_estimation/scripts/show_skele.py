import csv
import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

from IPython import embed

body_edges = [
    [0, 1], [0, 2], [0, 9], [0, 3],
    [9, 10], [10, 11],
    [3, 4], [4, 5],
    [2, 12], [12, 13], [13, 14],
    [2, 6], [6, 7], [7, 8],
    [1, 15], [15, 17],
    [1, 16], [16, 18]
]

def read_csv(path):
    data_skele = []
    csv_file = pd.read_csv(path)
    csv_file = np.array(csv_file)
    embed()
    #df = pd.DataFrame(csv_file(19, 3), index=data, columns=['x', 'y', 'z'])
    for id in range(len(csv_file) // 19):
        data_skele.append(csv_file.iloc[id * 19 : id * 19 + 19])
    return data_skele

def show_skele(pose):
    import mpl_toolkits.mplot3d.axes3d as p3

    fig = plt.figure()
    ax = fig.gca(projection='3d')

    X = np.array(pose[0]['x'])
    Y = np.array(pose[0]['y'])
    Z = np.array(pose[0]['z'])

    ax.scatter(X, Y, Z, marker='.', c='red')

    plt.show()

def main():
    data = read_csv('../data/skeletons.csv')
    show_skele(data)

if __name__=='__main__':
    main()