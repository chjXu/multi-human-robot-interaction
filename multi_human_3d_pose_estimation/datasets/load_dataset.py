import os.path as osp
import cv2
import glob
import numpy as np

import torch
from torch.utils.data.dataset import Dataset

class loadDataset(Dataset):
    def __init__(self, cfg):
        self.data_path = cfg.TEST_DATASET_PATH
        print(self.data_path)
        print(osp.join(self.data_path, '/*jpg'))
        
        self.image_list = glob.glob(osp.join(self.data_path, '/*.jpg'), recursive=True)
        self.list_size = len(self.image_list)
        print(self.list_size)

    def __getitem__(self, index):
        image_path = self.image_list[index].rstrip()
        image_name = image_path.replace(self.dataset_path, '').lstrip('/')
        image = cv2.imread(image_path, cv2.IMREAD_COLOR)
        self.image_shape = (image.shape[1], image.shape[0])

        return image_name

    def __len__(self):
        return self.list_size