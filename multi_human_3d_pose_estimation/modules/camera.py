import math
import os
import json
import numpy as np
import rospy
import tf
from IPython import embed


class Camera:
    def __init__(self, mode, camera_id):
        self.mode = mode
        if self.mode == 0:
            self.file_path = '/home/xuchengjun/multi_human/data/extrinsics.json'
            rospy.loginfo_once("The Camera params takes default!")
            self.R, self.t, self.K, self.D = self.read_camera_pose(self.file_path)
        elif mode == 1:
            rospy.loginfo_once("Read Camera_" + str(camera_id) + " parameters.")
            if camera_id == 1:
                self.file_path_in = '/home/xuchengjun/multi_human/data/my_camera_1_intrinsics.json'
                #self.file_path_ex = '/home/xuchengjun/multi_human/data/my_camera_extrinsics.json'
                self.K, self.D = self.read_my_cam_In_params(self.file_path_in)
                # if self.save_extrinsics(self.file_path_ex) == True:
                #     pass
                #self.R, self.t = self.read_my_cam_Ex_params(self.file_path_ex)
                # self.R, self.t = self.listen_tf()
            elif camera_id == 2:
                self.file_path_in = '/home/xuchengjun/multi_human/data/my_camera_2_intrinsics.json'
                self.K, self.D = self.read_my_cam_In_params(self.file_path_in)
            else:
                print("Please enter right camera_id.")
        else:
            rospy.logerr("Please enter valid mode number.")

    def getAllParams(self, scale):
        return self.K * scale, self.D * scale

    def read_camera_pose(self, file_path):
        if file_path is None:
            print('extrinsics file isnot exist!')

        with open(file_path, 'r') as f:
            extrinsics = json.load(f)

        R = np.array(extrinsics['R'], dtype=np.float32)
        t = np.array(extrinsics['t'], dtype=np.float32)
        K = np.array(extrinsics['K'], dtype=np.float32)
        D = np.array(extrinsics['D'], dtype=np.float32)

        # array([[0.13351353, 0.03326777, -0.9904884],
        #        [-0.0915409, 0.9955776, 0.02109909],
        #        [0.98681, 0.08785536, 0.1359685]], dtype=float32)

        return R, t, K, D

    def read_my_cam_In_params(self, file_path):
        if file_path is None:
            print ('extrinsics file isnot exist!')

        with open(file_path, 'r') as f:
            extrinsics = json.load(f)

        K = np.array(extrinsics['K'], dtype=np.float32)
        D = np.array(extrinsics['D'], dtype=np.float32)

        rospy.loginfo_once("Intrinsics has get.")
        return K, D

    def read_my_cam_Ex_params(self, file_path):
        if file_path is None:
            print('extrinsics file isnot exist!')

        with open(file_path, 'r') as f:
            extrinsics = json.load(f)

        R = np.array(extrinsics['R'], dtype=np.float32)
        t = np.array(extrinsics['t'], dtype=np.float32)

        rospy.loginfo_once("Extrinsics has get.")
        return R, t

    def listen_tf(self):

        # the two function is wrong
        def quat_to_rot(quat):
            q = quat[:]
            w = q[0]
            x = q[1]
            y = q[2]
            z = q[3]

            rot_matrix = np.array(
                [[1.0 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
                 [2 * x * y + 2 * z * w, 1.0 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w],
                 [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1.0 - 2 * x * x - 2 * y * y],
                 ]
            )
            return rot_matrix

        def trans_to_np(trans):
            assert len(trans) == 3
            _trans = np.zeros((3, 1), dtype=np.float32)
            for i in range(len(_trans)):
                _trans[i] = trans[i]
            return _trans

        _rot_matrix = np.zeros((3, 3), dtype=np.float32)
        _trans_matrix = np.zeros((3, 1), dtype=np.float32)
        listener = tf.TransformListener()
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                # quaternion and trans
                (trans, rot) = listener.lookupTransform('Camera_1', 'marker_0', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            rate.sleep()

            _rot_matrix = quat_to_rot(rot)
            _trans_matrix = trans_to_np(trans)

            if not _rot_matrix is None and not _trans_matrix is None:
                break
        rospy.loginfo_once("tf has been listen!")
        print(tf.transformations.quaternion_matrix(rot))
        return _rot_matrix, _trans_matrix

    def save_extrinsics(self, file_path):
        R, t = self.listen_tf()

        data = {"R":[], "t":[]}

        for i in range(len(R)):
            data['R'].append(R[i])
        for i in range(len(t)):
            data['t'].append(t[i])

        f = open(file_path, 'w')
        f.writelines(str(data))
        f.close()

        return True


