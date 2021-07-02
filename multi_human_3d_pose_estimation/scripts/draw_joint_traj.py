import csv
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
sys.path.append("/home/xuchengjun/multi_human")
from modules.one_euro_filter import OneEuroFilter

translation_filter = [OneEuroFilter(freq=80, beta=0.01),
                      OneEuroFilter(freq=80, beta=0.01),
                      OneEuroFilter(freq=80, beta=0.01)]

def filter(translation):
    filtered_translation = []
    for coordinate_id in range(3):
        filtered_translation.append(translation_filter[coordinate_id](translation[coordinate_id]))
    return filtered_translation

X = []
Y = []
Z = []
with open('../data/skeletons.csv', 'r') as f:
    reader = csv.reader(f)

    for row in reader:
        X.append(row[0])
        Y.append(row[1])
        Z.append(row[2])

X = np.array(X, dtype=np.float)
X = X.astype(np.int16)
Y = np.array(Y, dtype=np.float)
Y = Y.astype(np.int16)
Z = np.array(Z, dtype=np.float)
Z = Y.astype(np.int16)
#print(Z)



plt.plot(X[:50])
plt.plot(Y[:50])
plt.plot(Z[:50])
#print(Z[:50])
fig = plt.figure(figsize=(12, 10))
ax = fig.gca(projection='3d')
#x3d = Axes3D(fig)
figure = ax.plot(X[:50], Y[:50], Z[:50], c='r')
#ax3d.scatter(X,Y,Z,c="b",marker="*")

plt.show()