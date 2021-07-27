import argparse
import os
import rospy

import sys
sys.path.append('/home/xuchengjun/Public/zx_human')
import cv2
import numpy as np
import torch
#torch.multiprocessing.set_start_method('spawn')
import json
import csv
from numpy import mat
import codecs
import threading
# import message_filters

from models.with_mobilenet import PoseEstimationWithMobileNet
from pose_extractor import extract_poses
from modules.pose import Pose_2d, Pose_3d, draw, project_pose_from_camera, rotate_poses, compute_pose_3d

import time
# from action.recognizer import load_action_premodel, action_recognition, ActionClassifier
# 图像读入、视频读入、相机话题、图片队列读入
# from modules.input_reader import ImageReader, VideoReader, CameraTopic, ImageSeqReader
from modules.extractor_pose import parse_pose  #提取姿态
from modules.InferenceEngine import InferenceEngine #推理
from modules.camera import Camera
from threading import Lock, Thread
from IPython import embed
import math

camera = Camera(0, 0)
R, t, K, D = camera.get_camera_pose()
total_err = 0
err = 0

#数据集的相机参数
#旋转矩阵
# R = np.array([[0.1201471256,0.03330941579,-0.9921971332],
#              [-0.09157840245,0.9955474156,0.02233247834],
#              [0.9885231734,0.08818064529,0.1226625834]])
# #内参
# K = np.array([[1633.34,0,942.256],
#              [0,1628.84,557.344],
#              [0,0,1]])
# #平移向量
# t = np.array([23.27031863,
#              126.5360495,
#              284.0106483])
# #畸变系数
# dis = np.array([-0.220878,0.189272,7.79405e-05,0.000739643,0.0418043])

body_edges = np.array([[1,2],[1,4],[4,5],[5,6],[1,3],[3,7],[7,8],[8,9],[3,13],[13,14],[14,15],[1,10],[10,11],[11,12]])-1

#数据保存的路径
path = '../date.csv'
#设置标签json文件的路径
data_path = '/media/xuchengjun/datasets/panoptic-toolbox/171204_pose1_sample'
hd_skel_json_path = data_path+'/hdPose3d_stage1_coco19/'

uv = np.zeros((2,),dtype=np.float32)


def returnPose(pose_3d):
    pose = np.ones((19, 3), dtype=np.float32)
    for joint_id in range(pose_3d.shape[1]):
        pose[joint_id, 0] = pose_3d[0, joint_id]
        pose[joint_id, 1] = pose_3d[1, joint_id]
        pose[joint_id, 2] = pose_3d[2, joint_id]

    return pose

def project_pose_from_camera(x, K, Kd):
    #
    #         [x]   [f  0  cx  0]   [X_c]
    #   Z_c * [y] = [0  f  cy  0] * [Y_c]
    #         [1]   [0  0  1   0]   [Z_c]
    #                               [ 1 ]

    x[0:2,:] = x[0:2,:]/x[2,:]
    
    r = x[0,:]*x[0,:] + x[1,:]*x[1,:]
    
    x[0,:] = x[0,:]*(1 + Kd[0]*r + Kd[1]*r*r + Kd[4]*r*r*r) + 2*Kd[2]*x[0,:]*x[1,:] + Kd[3]*(r + 2*x[0,:]*x[0,:])
    x[1,:] = x[1,:]*(1 + Kd[0]*r + Kd[1]*r*r + Kd[4]*r*r*r) + 2*Kd[3]*x[0,:]*x[1,:] + Kd[2]*(r + 2*x[1,:]*x[1,:])

    x[0,:] = K[0,0]*x[0,:] + K[0,1]*x[1,:] + K[0,2]
    x[1,:] = K[1,0]*x[0,:] + K[1,1]*x[1,:] + K[1,2]
    
    return x

def projectPoints(X, K, R, t, Kd):
    """ Projects points X (3xN) using camera intrinsics K (3x3),
    extrinsics (R,t) and distortion parameters Kd=[k1,k2,p1,p2,k3].
    
    Roughly, x = K*(R*X + t) + distortion
    
    See http://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/camera_calibration.html
    or cv2.projectPoints
    """
    # pose_3d = transPose(pose_3d)
    # pose_3d = pose_3d.reshape((-1, 3)).transpose()
    # pose_3d = X[:3]
    R = mat(R)
    # x = np.asarray(R*pose_3d + t)
    # # x = pose_3d[0:3, :] = np.dot(R, pose_3d[0:3, :] + t)
    # x[0:2, :] = x[0:2, :]/x[2, :]
    # r = x[0, :] * x[0, :] + x[1, :] * x[1, :]

    # x[0, :] = x[0, :] * (1 + kd[0]*r + kd[1]*r*r + kd[4]*r*r*r) + 2*kd[2]*x[0, :]*x[1, :] + kd[3]*(r + 2*x[0, :]*x[0, :])
    # x[1, :] = x[1, :] * (1 + kd[0]*r + kd[1]*r*r + kd[4]*r*r*r) + 2*kd[3]*x[0, :]*x[1, :] + kd[2]*(r + 2*x[1, :]*x[1, :])

    # x[0, :] = K[0, 0]*x[0, :] + K[0, 1]*x[1, :] + K[0, 2]
    # x[1, :] = K[1, 0]*x[0, :] + K[1, 1]*x[1, :] + K[1, 2]

    # return x[:2]
    x = np.asarray(R*X + t)
    
    x[0:2,:] = x[0:2,:]/x[2,:]
    
    r = x[0,:]*x[0,:] + x[1,:]*x[1,:]
    
    x[0,:] = x[0,:]*(1 + Kd[0]*r + Kd[1]*r*r + Kd[4]*r*r*r) + 2*Kd[2]*x[0,:]*x[1,:] + Kd[3]*(r + 2*x[0,:]*x[0,:])
    x[1,:] = x[1,:]*(1 + Kd[0]*r + Kd[1]*r*r + Kd[4]*r*r*r) + 2*Kd[3]*x[0,:]*x[1,:] + Kd[2]*(r + 2*x[1,:]*x[1,:])

    x[0,:] = K[0,0]*x[0,:] + K[0,1]*x[1,:] + K[0,2]
    x[1,:] = K[1,0]*x[0,:] + K[1,1]*x[1,:] + K[1,2]
    
    return x


def projection_to_camera(X,R,t):
    R = mat(R)
    x = np.asarray(R*X + t)
    return x


def draw_skele(img, pose_2d):
    for joint_id in range(pose_2d.shape[1]):
        joint = pose_2d[:, joint_id]

        cv2.circle(img, (int(joint[0]), int(joint[1])), 3, [255, 0, 0], 4)
    cv2.imshow("img", img)
    cv2.waitKey(3)

def compute_MPJPE(pt,pt_gt,err):  #pt:(19,3) pt_gt:(3,19)
    x = []
    y = []
    z = []
    pt_com = []
    
    for joint in pt:
        x.append(joint[0])
        y.append(joint[1])
        z.append(joint[2])
    
    pt_com.append(x)
    pt_com.append(y)
    pt_com.append(z)

    # print(pt_com)

    # print(len(pt_com),len(pt_gt))

    #计算
    for id in range(15):
        x_err = (pt_com[0][id] - pt_gt[0][0][id]) ** 2
        y_err = (pt_com[1][id] - pt_gt[0][1][id]) ** 2
        z_err = (pt_com[2][id] - pt_gt[0][2][id]) ** 2
        err += math.sqrt(x_err + y_err + z_err)
    
    print(err/15)
    
    return err / 15            #除以一帧中的所有关键点
    

def run_demo(net, image_path, net_input_height_size, cam_mode, camera_id,image_index,ite):
    

    global err
    global total_err
    img = cv2.imread(image_path)
    # 相机读取内外参数(have been setted as the dataset's )
    stride = 8
    fx = K[0, 0]

    com_poses_3d_cam = None
    is_video = False

    if img is None:
        print("Image empty")

    orig_img = img.copy()

    input_scale = net_input_height_size / img.shape[0]
    scale_img = cv2.resize(img, dsize=None, fx=input_scale, fy=input_scale)
    scale_img = scale_img[:, 0:scale_img.shape[1] - (scale_img.shape[1] % stride)]
    if fx == 0:
        fx = np.float32(0.8 * img.shape[1])
    # net = net.net.cuda("cuda:{}".format(camera_id))
    
    inference_results = net.infer(scale_img)  # 这个是3D网络推出来的
    # 姿势解析
    current_poses_3d, current_poses_2d, pose_depth = parse_pose(inference_results, stride, input_scale, fx,
                                                                is_video=is_video)
    
    # print('******current_poses_2d*******')
    # print(current_poses_2d)

    # if (len(current_poses_3d)):
    #     current_poses_3d = current_poses_3d.reshape(current_poses_3d.shape[0], 19, -1)[:, :, 0:3]

    # human_list = HumanList()  #储存所有人的列表
    #compute the 3D pose(camera) from pose_2d 利用openpose得到的信息进行计算 depth + pose_2D
    #计算并保存数据
   
    compute_pose = np.zeros((3,19))
    for pose_id in range(len(current_poses_2d)):
        com_poses_3d_cam = compute_pose_3d(current_poses_2d[pose_id], pose_depth[pose_id], K)
        

        for joint_id in range(len(com_poses_3d_cam)):
            if com_poses_3d_cam[joint_id][3] == -1:
                com_poses_3d_cam[joint_id, :] = current_poses_3d[pose_id, joint_id]
    # print(com_poses_3d_only)


    #计算数据并保存
    # with open(path,'a+') as file:
    #     writer = csv.writer(file,)
    #     writer.writerow(com_poses_3d_only)
    # file.close()
    
    print('==============================================')

    try:
        #load the json file with the current frame's skeleton
        ske_json_frame = hd_skel_json_path+'body3DScene_{0:08d}.json'.format(image_index)
        print(ske_json_frame)
        with open(ske_json_frame) as dfile:
            bframe = json.load(dfile)
        # Show only points detected with confidence
            # valid = skel[3,:]>0.1
    except:
        print('the json file load failed')

    #先将3D坐标进行重投影
    
    all_skele = []
    all_2d_pose = []
    cam_3d_pose = []
    for body in bframe['bodies']:
        skel = np.array(body['joints19']).reshape((-1,4)).transpose()
        all_skele.append(skel)
 
    for skele in all_skele:
        # print(skel[i][:3])
        # print(skel[0:3,i])

        #重投影回2D

        # pt_pixel = projectPoints(skele[:3],K,R,t,D)  #[:3] 前三行
        # all_2d_pose.append(pt_pixel)

        #重投影回cam_3d
        pt_cam = projection_to_camera(skel[:3],R,t)
        #pt_pix = project_pose_from_camera(pt_cam, K, D)
        # print(skele)
        # print(pt_cam)
        # #print(pt_pix)
        # print(com_poses_3d_only)
        cam_3d_pose.append(pt_cam)
        #all_2d_pose.append(pt_pix)


    #print(cam_3d_pose)
    for pose_2d in all_2d_pose:
        draw_skele(img, pose_2d)

    current_poses_3d = current_poses_3d.reshape(current_poses_3d.shape[0], 19, -1)[:, :, 0:3]
    current_poses_3d = current_poses_3d[0]
    
    err = compute_MPJPE(current_poses_3d,cam_3d_pose,err)
    total_err += err   #计算所有帧 
    
    if (image_index + 1) == ite:
        MPJPE = total_err / ite
        print("MPJPE:{:03f}".format(MPJPE))


if __name__ == "__main__":

    
    total_err = 0

    parser = argparse.ArgumentParser()
    parser.add_argument('--model',type=str,default='../../human_pose_3d.pth')
    parser.add_argument('--image_mode',type=bool,default=1) #只是为了读取图片
    # parser.add_argument('--image_path',type=str,default='',help='path to image file')
    parser.add_argument('--device',type=str,default='GPU')
    parser.add_argument('--height-size',type=int,default=256)
    parser.add_argument('--track',type=int,default=1)
    parser.add_argument('--recog_action',type=int,default=1)
    parser.add_argument('--cam_mode',type=int,default=1)
    arg = parser.parse_args()

    #返回网络模型
    net = InferenceEngine(arg.model,arg.device,1)
    #存放图片的文件夹
    img_dir = '/media/xuchengjun/datasets/panoptic-toolbox/171204_pose1_sample/hdImgs'
    #循环，处理所有的图片
    if arg.image_mode:
        #图片列表
        image_list = []
        img_list = []
        for root,s_dirs,_ in os.walk(img_dir,topdown=True):
            for sub_dir in s_dirs:
                i_dir = os.path.join(root,sub_dir)
                img_list = os.listdir(i_dir)  #所有图片的名称
                for i in range(len(img_list)):
                    if not img_list[i].endswith('jpg'):
                        continue
                    img_path = os.path.join(i_dir,img_list[i])
                    image_list.append(img_path)
        # print(image_list)

        if image_list is None:
            raise ValueError('do not found any picture')
        # # image_frame = ImageReader(arg.image)
        # image_frame = ImageSeqReader(image_list)

        # img = cv2.imread(image_list[0])

        # image_index = 0
        # for image_path in image_list[:1]:
        #     if image_index < image_list:
        #         i
        #         # print('good')
        #         run_demo(net, image_path, arg.height_size, arg.cam_mode, 1,image_index)
        #         image_index += 1
        ite = len(image_list)
        # print(ite)
        for image_index in range(ite):
            
            image_path = img_dir + '/00_00/00_00_{0:08d}.jpg'.format(image_index)
            print(image_path)
            run_demo(net,image_path,arg.height_size,arg.cam_mode,1,image_index,ite)


