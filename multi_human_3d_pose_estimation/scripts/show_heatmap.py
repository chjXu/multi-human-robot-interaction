import cv2
import numpy as np
from PIL import Image
from pyheatmap.heatmap import HeatMap
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from matplotlib.colors import LogNorm
import matplotlib.image as im
from IPython import embed

def heatmap():
    N = 10000
    x = np.random.rand(N) * 255
    y = np.random.rand(N) * 255
    data = []

    for i in range(N):
        tmp = [int(x[i]), int(y[i]), 1]
        data.append(tmp)
    heat = HeatMap(data)
    heat.clickmap(save_as='1.png')
    heat.heatmap(save_as='2.png')

def loca():
    x = np.random.rand(10)
    y = np.random.rand(10)

    print(xmin, xmax, ymin, ymax)

    fig, ax = plt.subplots(ncols=1, sharey=True, figsize=(7, 7))
    # fig.subplots_adjust(hspace=0.5, left=0.07, right=0.93)
    # ax = axs[0]
    hb = ax.hexbin(x, y, gridsize=50, bins=[-1, 1], cmap='BuGn')
    ax.set(xlim=(xmin, xmax), ylim=(ymin, ymax))
    ax.set_title("XXX")
    cb = fig.colorbar(hb, ax=ax)
    cb.set_label('distance')

    # ax = axs[1]
    # hb = ax.hexbin(x, y, gridsize=50, bins='log', cmap='inferno')
    # ax.set(xlim=(xmin, xmax), ylim=(ymin, ymax))
    # ax.set_title("XXX")
    # cb = fig.colorbar(hb, ax=ax)
    # cb.set_label('log')

    plt.show()

def show_location_map(img):
    img = im.imread('../data/map_np.jpg')
    plt.imshow(img)

    lum_img = img[:, :, 0]
    plt.imshow(lum_img)
    plt.colorbar()

    plt.show()
    # img = cv2.imread(img)
    # img = cv2.resize(img, (1920, 1080))
    # cv2.imshow('heat', img * 255)
    # cv2.waitKey(0)

def main():
    source_img = cv2.imread('../multiple.jpg')
    heat_map = cv2.imread('../data/heatmap.jpg')
    paf_map = cv2.imread('../data/pafs.jpg')
    if heat_map is None or paf_map is None:
        print("Image load error!")

    cv2.imshow('source', source_img)
    cv2.imshow('heatmap', heat_map)
    cv2.imshow('pafmap', paf_map)
    dst_heat = cv2.addWeighted(source_img, 0.2, heat_map, 0.8, 0)
    dst_paf = cv2.addWeighted(source_img, 0.2, paf_map, 0.8, 0)
    dst_heat = cv2.applyColorMap(dst_heat, cv2.COLORMAP_JET)
    dst_paf = cv2.applyColorMap(dst_paf, cv2.COLORMAP_JET)
    cv2.imshow('heat', dst_heat)
    cv2.imshow('paf', dst_paf)
    cv2.waitKey(0)

if __name__=='__main__':
    main()
    # show_location_map('../data/map_np.jpg')
    # heatmap()