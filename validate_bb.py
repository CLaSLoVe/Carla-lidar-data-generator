import pickle
from tools import *
import cv2
import os
import numpy as np
from tqdm import tqdm
# import cupy as cp
import csv


THRESHOLD = [10, 8, 5]

# with open(r'D:\jd\WindowsNoEditor\PythonAPI\my\data\2022-07-06 17.13.57\1.6\posit\0000749463.pkl', 'rb') as f:
#     posit = pickle.load(f)
#
# points = np.fromfile(r'D:\jd\WindowsNoEditor\PythonAPI\my\data\2022-07-06 17.13.57\1.6\lidar\0000749463.bin', dtype="float32").reshape((-1, 4))
#
# pic = cv2.imread(r'D:\jd\WindowsNoEditor\PythonAPI\my\data\2022-07-06 17.13.57\1.6\pics\0000749463.png')

def validate(posit_path, points_path, pic_path, label_path):
    with open(posit_path, 'rb') as file:
        f = pickle.load(file)
    local_lidar_points = np.fromfile(points_path, dtype="float32").reshape((-1, 4))
    pic = cv2.imread(pic_path)


    ego_x = f['x']
    ego_y = f['y']
    ego_z = f['z']
    ego_yaw = f['yaw']
    actor_dict = f['dict']
    labels = []
    for i in actor_dict.keys():
        actor = actor_dict[i]

        draw = actor['draw']

        bb_sensor = actor['bb'][:, :3]
        Oz = np.array(bb_sensor[0] + bb_sensor[2]) / 2
        Ox = np.array(bb_sensor[5] + bb_sensor[2]) / 2
        Oy = np.array(bb_sensor[5] + bb_sensor[0]) / 2
        O  = np.array(bb_sensor[6] + bb_sensor[0]) / 2

        Px = unit_vector(Ox - O)
        Py = unit_vector(Oy - O)
        Pz = unit_vector(Oz - O)
        #
        actor_co = np.array([Px, Py, Pz]).T
        vecs = local_lidar_points[:,:3] - O
        p = np.dot(np.array(vecs), np.array(actor_co))
        p = np.absolute(p)
        l = actor['l']  # *1.15
        w = actor['w']  # *1.15
        h = actor['h']  # *1.15
        valid_array = np.logical_and(np.logical_and(p[:, 0] <= l, p[:, 1] <= w), p[:, 2] <= h)
        ans = np.sum(valid_array)
        L = np.sqrt(O[0]**2+O[1]**2)


        if L < 50:
            v = THRESHOLD[0]
        elif L < 100:
            v = THRESHOLD[1]
        else:
            v = THRESHOLD[2]

        if actor['type'][0] == 'w':
            v = v / 2
            if ans >= v:
                actor['valid'] = 1
        else:
            if ans >= v:
                actor['valid'] = 1

        if actor['valid']:
            yaw = degree2radian(actor['yaw'] - ego_yaw)

            dx = O[0]
            dy = O[1]
            dz = O[2]

            if dx == 0:
                alpha = 90
            else:
                alpha = np.degrees(np.arctan(dy/dx))
            if dx < 0:
                alpha += 180
            alpha = degree2radian(alpha)

            string = [classify_actor(actor),
                      1,
                      1,
                      alpha,
                      1,
                      1,
                      1,
                      1,
                      h * 2,
                      w * 2,
                      l * 2,
                      dx,
                      dy,
                      dz,
                      yaw,
                      1]
            labels.append(string)

            for i in range(4):
                cv2.line(pic, (int(draw[1][i]), int(draw[0][i])), (int(draw[1][i + 1]), int(draw[0][i + 1])), (0, 255, 0), 1, 4)
    cv2.imwrite(pic_path, pic)
    np.savetxt(label_path, labels, fmt='%s')

    return labels

# ROOT = 'D:\jd\WindowsNoEditor\PythonAPI\my\data\\2022-07-06 17.35.28\\1.6\\'
#
# NAME = '0001664263'
#
# labels = validate(ROOT+'posit\\'+NAME+'.pkl',
#                   ROOT+'lidar\\'+NAME+'.bin',
#                   ROOT+'pics\\'+NAME+'.png',
#                   ROOT+'label\\'+NAME+'.txt',)
#
#
# def batch_validate(PATH):
#     for i in [0.4, 0.8, 1.6]:
#         ROOT = PATH+'\\'+str(i)+'\\'
#         path = PATH+'\\'+str(i)+'\\lidar'
#         frames = [x.split('.')[0] for x in os.listdir(path)]
#         for f, frame in enumerate(tqdm(frames)):
#             labels = validate(ROOT+'posit\\'+frame+'.pkl',
#                               ROOT+'lidar\\'+frame+'.bin',
#                               ROOT+'pics\\'+frame+'.png',
#                               ROOT+'label\\'+frame+'.txt',)
#             # print(i, ':', f, ':', frame)
#     return PATH
#
#
# if __name__ == '__main__':
#     # PATH = 'D:\jd\WindowsNoEditor\PythonAPI\my\data\\2022-07-06 17.35.28'
#     ROOT = 'D:\jd\WindowsNoEditor\PythonAPI\my\data\\2022-07-07 10.19.16\\0.8\\'
#     # batch_validate(PATH)
#     NAME = '0000040962'
#     labels = validate(ROOT+'posit\\'+NAME+'.pkl',
#                       ROOT+'lidar\\'+NAME+'.bin',
#                       ROOT+'pics\\'+NAME+'.png',
#                       ROOT+'label\\'+NAME+'.txt',)


