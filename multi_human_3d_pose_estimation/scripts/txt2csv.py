import numpy as np
import os
import csv

def txt2csv(txt_file):
    # read txt file
    with open(txt_file, 'r') as f:
        for line in f.readlines():
            line_list = filter(lambda ch: ch not in '[', line)
            print(line_list)

if __name__=='__main__':
    folder_path = '../data'
    txt_file_list = [os.path.join(folder_path, file) for file in os.listdir(folder_path) if
                     os.path.join(folder_path, file).endswith('.txt')]

    for txt_file in txt_file_list:
        if txt_file == '../data/test_txt.txt':
            txt2csv(txt_file)
