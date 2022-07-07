import pickle
from tools import *
import cv2
import os
import numpy as np
from tqdm import tqdm
# import cupy as cp
import csv
from multiprocessing import Process
from validate_bb import validate

# ROOT = 'D:\jd\WindowsNoEditor\PythonAPI\my\data\\2022-07-06 17.35.28\\1.6\\'
#
# NAME = '0001664263'
#
# labels = validate(ROOT+'posit\\'+NAME+'.pkl',
#                   ROOT+'lidar\\'+NAME+'.bin',
#                   ROOT+'pics\\'+NAME+'.png',
#                   ROOT+'label\\'+NAME+'.txt',)


def get_frames(PATH):
    for i in [0.4]:
        path = PATH+'\\'+str(i)+'\\lidar'
        frames = [x.split('.')[0] for x in os.listdir(path)]
    return frames


def batch_validate(PATH, h, frames):
    ROOT = PATH+'\\'+str(h)+'\\'
    for f, frame in enumerate(tqdm(frames)):
        labels = validate(ROOT+'posit\\'+frame+'.pkl',
                          ROOT+'lidar\\'+frame+'.bin',
                          ROOT+'pics\\'+frame+'.png',
                          ROOT+'label\\'+frame+'.txt',)
    return PATH


if __name__ == '__main__':
    # PATH = 'D:\jd\WindowsNoEditor\PythonAPI\my\data\\2022-07-06 17.35.28'
    PATH = 'D:\jd\WindowsNoEditor\PythonAPI\my\data\\2022-07-07 10.19.16'
    frames = get_frames(PATH)

    N = 16
    n = int(np.ceil(len(frames)/N))

    p_list = []
    for h in [0.4, 0.8, 1.6]:
        for i in range(N):
            end = int(min((i+1)*n, len(frames)))
            p = Process(target=batch_validate, args=(PATH, h, frames[i*n:end]))
            p.start()
            p_list.append(p)
    for p in p_list:
        p.join()



