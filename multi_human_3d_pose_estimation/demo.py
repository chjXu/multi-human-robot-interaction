import argparse
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from visualization_msgs.msg import MarkerArray, Marker
from human_pose_msgs.msg import PointWithProb, Human, HumanList
import sys

import cv2
import numpy as np
import torch
#torch.multiprocessing.set_start_method('spawn')
import json
import csv
import codecs
import threading
import message_filters

from models.with_mobilenet import PoseEstimationWithMobileNet
from modules.slow_extractor import extract_keypoints, group_keypoints
from modules.pose import Pose_2d, Pose_3d, draw, project_pose_from_camera, rotate_poses, compute_pose_3d
from action.data_processing import *
from action.recognizer import load_model, predict

import time
#from action.recognizer import load_action_premodel, action_recognition, ActionClassifier
from modules.input_reader import ImageReader, VideoReader, CameraTopic, ImageSeqReader
from modules.extractor_pose import parse_pose
from modules.InferenceEngine import InferenceEngine
from modules.camera import Camera
from threading import Lock, Thread
from IPython import embed

# add here 2021.1.08
from geometry_msgs.msg import Point  # add
import modules.pose as pose
import multiprocessing.spawn
from DNN_printer import DNN_printer
###########


#import torchvision.models as models



class Com_Thread(threading.Thread):
    def __init__(self, poses_2d, poses_depth, R, t, K, D, img):
        super(Com_Thread, self).__init__()
        lock = Lock()

        lock.acquire()
        self.current_poses_2d = poses_2d
        self.poses_depth = poses_depth
        self.R = R
        self.t = t
        self.K = K
        self.D = D
        self.image = img
        lock.release()

    def run(self):
    # lock.acquire()
    # current_poses_2d = poses_2d
    # lock.release()

        for pose_id in range(len(self.current_poses_2d)):
            com_poses_3d = compute_pose_3d(self.current_poses_2d[pose_id], self.poses_depth[pose_id], self.R, self.t, self.K)
            re_pose = project_pose(com_poses_3d, self.R, self.t, self.K, self.D)
            draw(re_pose, self.image, 'recom_project')

# this function will return pose from an image
def get_pose(net, img, cam_mode, camera_id):
    posePub = rospy.Publisher('HumanPose_3d_' + str(camera_id), HumanList, queue_size=10)

    K, D = Camera(cam_mode, camera_id).getAllParams(1.0)
    stride = 8
    fx = K[0, 0]
    input_scale = 256 / img.shape[0]
    scale_img = cv2.resize(img, dsize=None, fx=input_scale, fy=input_scale)
    scale_img = scale_img[:, 0:scale_img.shape[1] - (scale_img.shape[1] % stride)]
    if fx == 0:
        fx = np.float32(0.8 * img.shape[1])

    inference_results = net.infer(scale_img)
    current_poses_3d, current_poses_2d, pose_depth = parse_pose(inference_results, stride, input_scale, fx)

    # if (len(current_poses_3d)):
    #     current_poses_3d = current_poses_3d.reshape(current_poses_3d.shape[0], 19, -1)[:, :, 0:3]
    # for pose_3d in current_poses_3d:
    #     re_pose = project_pose_from_camera(pose_3d, K, D)
    #     draw(re_pose, img, 'reproject')

    # human_list = HumanList()
    # # compute the 3D pose from pose_2d
    # for pose_id in range(len(current_poses_2d)):
    #     com_poses_3d_cam = compute_pose_3d(current_poses_2d[pose_id], pose_depth[pose_id], K)

    #     for joint_id in range(len(com_poses_3d_cam)):
    #         if com_poses_3d_cam[joint_id][3] == -1:
    #             com_poses_3d_cam[joint_id, :3] = current_poses_3d[pose_id, joint_id]
    #             com_poses_3d_cam[joint_id, 3] = 0.5

    #     re_pose = project_pose_from_camera(com_poses_3d_cam, K, D)
    #     draw(re_pose, img, 'recom_project')

    #     human = Human()
    #     # com_poses_3d_cam= com_poses_3d_cam[:, :3]
    #     for i in range(len(com_poses_3d_cam)):
    #         point = PointWithProb()
    #         point.x = com_poses_3d_cam[i][0]
    #         point.y = com_poses_3d_cam[i][1]
    #         point.z = com_poses_3d_cam[i][2]
    #         point.p = com_poses_3d_cam[i][3]
    #         human.body_key_points_prob.append(point)
    #     human.human_id = pose_id
    #     human_list.human_list.append(human)
    #     posePub.publish(human_list)
    return current_poses_3d, current_poses_2d



def run_demo(net, image_provider, net_input_height_size, track, recog_action, cam_mode, camera_id):
    posePub = rospy.Publisher('HumanPose_3d_' + str(camera_id), HumanList, queue_size=2)

    K, D = Camera(cam_mode, camera_id).getAllParams(1.0)

    # if track:
    #     print("tracking mode...")

    action_classifier = load_model("./action/action.pkl")
    stride = 8
    fx = K[0, 0]

    # previous_poses_3d_tracking = []
    com_poses_3d_cam = None
    # markerarray = MarkerArray()
    data_name = './data/skeletons.csv'
    delay = 33
    is_video = True
    # send_to_world = False
    # fourcc = cv2.VideoWriter_fourcc(*'XVID')
    # out = cv2.VideoWriter('./opeator.avi',fourcc, 20.0, (1920,1080))

    frame_total = 0
    cost_time=0

    print("ready")
    while not rospy.is_shutdown():
        for img in image_provider:
            start_time = time.time()

            if img is None:
                print("Image empty")
                
            orig_img = img.copy()
            frame_total += 1

            input_scale = net_input_height_size / img.shape[0]
            scale_img = cv2.resize(img, dsize=None, fx=input_scale, fy=input_scale)

            scale_img = scale_img[:, 0:scale_img.shape[1] - (scale_img.shape[1] % stride)]
            if fx == 0:
                fx = np.float32(0.8 * img.shape[1])
            # net = net.net.cuda("cuda:{}".format(camera_id))
            inference_results = net.infer(scale_img)
            current_poses_3d, current_poses_2d, pose_depth = parse_pose(inference_results, stride, input_scale, fx, is_video=is_video)
            # print(current_poses_3d)

            if(len(current_poses_3d)):
                # current_poses_3d = rotate_poses(current_poses_3d, R, t)
                current_poses_3d = current_poses_3d.reshape(current_poses_3d.shape[0], 19, -1)[:, :, 0:3]

            # if recog_action:
            #     for human in range(len(current_poses_2d)):
            #         action_recognition(current_poses_2d[human], action_classifier)

            # draw 2D pose from location-map
            # for pose_3d in current_poses_3d:
            #    re_pose = project_pose_from_camera(pose_3d, K, D)

            #    np.delete(re_pose,2)
            #    np.delete(re_pose,0)
            #     #re_pose.pop([index=2])
            #    normal_pose = pose_normalization(re_pose[:, :2])
            #    action = predict(normal_pose, action_classifier)
                # re_pose = project_pose(pose_3d, R, t, K, D)
                # draw(re_pose, img, 'reproject')

            # # draw 2D pose from Openpose
            # # for pose_2d in current_poses_2d:
            # #     draw(pose_2d, img, 'norm')
 




            # human_list = HumanList()
            # for pose_3d in current_poses_3d:
            #     re_pose = project_pose_from_camera(pose_3d, K, D)

            #     np.delete(re_pose,2)
            #     np.delete(re_pose,0)
            #     #re_pose.pop([index=2])
            #     normal_pose = pose_normalization(re_pose[:, :2])
            #     action = predict(normal_pose, action_classifier)

            #     human = Human()
            #     for joint in range(len(pose_3d)):
            #         point = PointWithProb()
            #         point.x = pose_3d[joint][0]
            #         point.y = pose_3d[joint][1]
            #         point.z = pose_3d[joint][2]
            #         point.p = 1.0

            #         human.body_key_points_prob.append(point)
            #     human.human_id = 0
            #     human.action = action
            #     human_list.human_list.append(human)
            #     posePub.publish(human_list)






            human_list = HumanList()
            # compute the 3D pose from pose_2d
            for pose_id in range(len(current_poses_2d)):
                com_poses_3d_cam = compute_pose_3d(current_poses_2d[pose_id], pose_depth[pose_id], K)

                for joint_id in range(len(com_poses_3d_cam)):
                    if com_poses_3d_cam[joint_id][3] == -1:
                        com_poses_3d_cam[joint_id, :3] = current_poses_3d[pose_id, joint_id]
                        com_poses_3d_cam[joint_id, 3] = 0.0
            
                re_pose = project_pose_from_camera(com_poses_3d_cam, K, D)
                draw(re_pose, img, 'recom_project')

                np.delete(re_pose,2)
                np.delete(re_pose,0)
                #re_pose.pop([index=2])
                normal_pose = pose_normalization(re_pose[:, :2])
                action = predict(normal_pose, action_classifier)

                human = Human()
                # com_poses_3d_cam= com_poses_3d_cam[:, :3]
                for i in range(len(com_poses_3d_cam)):
                    point = PointWithProb()
                    point.x = com_poses_3d_cam[i][0]
                    point.y = com_poses_3d_cam[i][1]
                    point.z = com_poses_3d_cam[i][2]
                    point.p = com_poses_3d_cam[i][3]

                    #if i == 9:
                    #    with open(data_name, "a+") as file:
                    #        writer = csv.writer(file)
                    #        writer.writerow(com_poses_3d_cam[9][:3])
                  
                    human.body_key_points_prob.append(point)
                human.human_id = pose_id
                human.action = action
                #human_list.humans_num = len(current_poses_3d)
                human_list.human_list.append(human)
                posePub.publish(human_list)






                # with open(data_name, "a+") as file:
                #     writer = csv.writer(file)
                #     writer.writerow(com_poses_3d_cam)
             
                # with open("./data/operate.txt", 'a+') as file:
                #     # point_list = []
                #     # for idx in range(re_pose.shape):
                #     #     point_list.append(float(format(re_pose[idx][0]), "0.3f")
                #     #     point_list.append(float(format(re_pose[idx][1]), "0.3f")
                #     file.write(str(normal_pose))
                #     file.write('\n')
                #     file.close()

            

            end_time = time.time()
            #cost_time += end_time - start_time
            print("working")
            #print("working in {} ms".format(cost_time/frame_total))
            cv2.putText(img, 'FPS: {}'.format(int(1/(end_time-start_time))), (30, 40), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255))
            img = cv2.addWeighted(orig_img, 0.5, img, 0.5, 0)
            # out.write(orig_img)
            cv2.imshow("Camera_" + str(camera_id), img)
            key = cv2.waitKey(delay)
            if key == 27:  # esc
                return
            elif key == 112:  # 'p'
                if delay == 33:
                    delay = 0
                else:
                    delay = 33



class TwoCameraTopic(object):
    def __init__(self, topic_name_1, topic_name_2, net_1, net_2):
        # rospy.init_node('Image_sub', anonymous=True)
        self.image_1 = None
        self.image_2 = None
        self.cv_bridge = CvBridge()
        self.topic_1 = topic_name_1
        self.topic_2 = topic_name_2
        self.image_1_sub = message_filters.Subscriber(self.topic_1, Image)
        self.image_2_sub = message_filters.Subscriber(self.topic_2, Image)
        self.net_1 = net_1
        self.net_2 = net_2

    def callback(self, image_1, image_2):
        self.image_1 = self.cv_bridge.imgmsg_to_cv2(image_1, "bgr8")
        self.image_2 = self.cv_bridge.imgmsg_to_cv2(image_2, "bgr8")

        # start_time = time.time()
        poses_3d_1, poses_2d_1 = get_pose(net_1, self.image_1, 1, 1)
        poses_3d_1, poses_2d_1 = get_pose(net_1, self.image_2, 1, 2)
        # cam1 = multiprocessing.Process(target=get_pose, args = (self.net_1, self.image_1, 1, 1))
        # cam2 = multiprocessing.Process(target = get_pose, args = (self.net_2, self.image_2, 1, 2))
        # 
        # cam1.start()
        # cam2.start()
        # #
        # cam1.join()
        # cam2.join()

        # end_time = time.time()
        # cv2.putText(self.image_1, 'FPS: {}'.format(int(1 / (end_time - start_time))), (30, 40), cv2.FONT_HERSHEY_COMPLEX, 1,
        #            (0, 0, 255))
        #cv2.putText(self.image_2, 'FPS: {}'.format(int(1 / (end_time - start_time))), (30, 40), cv2.FONT_HERSHEY_COMPLEX, 1,
         #           (0, 0, 255))

        #cv2.imshow('Camera_1', self.image_1)
        #cv2.imshow('Camera_2', self.image_2)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown('Quit')
            cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('human_detection', anonymous=True)

    rate = rospy.Rate(60)

    parser = argparse.ArgumentParser(
        description='''Lightweight human pose estimation python demo.
                       This is just for quick results preview.
                       Please, consider c++ demo for the best performance.''')
    parser.add_argument('--model', type=str, required=False, default='./human_pose_3d.pth',
                         help='path to the checkpoint')
    parser.add_argument('--mode', type=int, required=True, help="choose run mode. '1' means a picture, "
                        "'2' means usb_cam, '3' means ros topic")
    parser.add_argument('--cam_mode', type=int, required=False, default=1, help="choose cam mode. "
                        "'0' means default params, '1' means usb cam params")
    parser.add_argument('--height-size', type=int, default=256, help='network input layer height size')
    parser.add_argument('--video', type=str, default='', help='path to video file or camera id')
    parser.add_argument('--image', type=str, default='', help='path to image file')
    #multi thread for human pose estimation
    parser.add_argument('--topic_1', type=str, default='', help='path to camera topic')
    parser.add_argument('--topic_2', type=str, default='', help='path to camera topic')
    parser.add_argument('--device', type=str, default='GPU', help='run network inference on your device')
    parser.add_argument('--track', type=int, default=1, help='track pose id in video')
    parser.add_argument('--smooth', type=int, default=1, help='smooth pose keypoints')
    parser.add_argument('--recog_action', type=int, default=1, help='human action recognition')
    parser.add_argument('--is_video', type=int, default=True, help='video filter')
    args = parser.parse_args()

    args.cpu = True
    args.cam_mode = 1
    #
    net = InferenceEngine(args.model, args.device, 0)
    #net_1 = InferenceEngine(args.model, args.device, 0)
    #net_2 = InferenceEngine(args.model, args.device, 1)

    if args.mode == 1:
        args.image = './images/multiple.jpg'
        if args.image == '':
            raise ValueError('Image has to be provided')
        frame_provider = ImageReader(args.image)

        try:
            run_demo(net, frame_provider, args.height_size, args.track, args.recog_action, args.cam_mode, 1)
        except:
            raise IOError('Exits')
    elif args.mode == 2:
        if args.video == '':
            raise ValueError('Video has to be provided')
        frame_provider = VideoReader(args.video)
        
        try:
            run_demo(net, frame_provider, args.height_size, args.track, args.recog_action, args.cam_mode, 1)
        except:
            raise IOError('Exits')
            
    elif args.mode == 3:
        if args.topic_1 == '':
            raise ValueError('Topic empty...')
        frame_provider = CameraTopic(args.topic_1)

        try:
            run_demo(net, frame_provider, args.height_size, args.track, args.recog_action, args.cam_mode, 1)

        except:
            raise IOError('Exits')
    #elif args.mode == 4:
    #    if args.topic_1 == '' or args.topic_2 == '':
    #        raise ValueError('Please enter two image topic name...')
    #    frame_provider = TwoCameraTopic(args.topic_1, args.topic_2, net_1, net_2)

    #    try:
    #        ts = message_filters.ApproximateTimeSynchronizer([frame_provider.image_1_sub, frame_provider.image_2_sub],
    #                                                         10,
    #                                                         0.1)
    #        ts.registerCallback(frame_provider.callback)
    #        rospy.spin()

    #    except KeyboardInterrupt:
    #        print("shutting down")
    else:
        raise IOError('Please right num to choose mode')






