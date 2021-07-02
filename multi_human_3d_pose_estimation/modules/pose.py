import cv2
import csv

import roslib
roslib.load_manifest('easy_markers')
import rospy
from easy_markers.generator import *
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point  # add


import numpy as np
from numpy import mat
from modules.camera import Camera

from modules.one_euro_filter import OneEuroFilter
from IPython import embed

BODY_PARTS_KPT_IDS = [[1, 2], [1, 5], [2, 3], [3, 4], [5, 6], [6, 7], [1, 8], [8, 9], [9, 10], [1, 11],
                      [11, 12], [12, 13], [1, 0], [0, 14], [14, 16], [0, 15], [15, 17], [2, 16], [5, 17]]
BODY_PARTS_PAF_IDS = ([12, 13], [20, 21], [14, 15], [16, 17], [22, 23], [24, 25], [0, 1], [2, 3], [4, 5],
                      [6, 7], [8, 9], [10, 11], [28, 29], [30, 31], [34, 35], [32, 33], [36, 37], [18, 19], [26, 27])

body_edges = [
    [0, 1], [0, 2], [0, 9], [0, 3],
    [9, 10], [10, 11],
    [3, 4], [4, 5],
    [2, 12], [12, 13], [13, 14],
    [2, 6], [6, 7], [7, 8],
    [1, 15], [15, 17],
    [1, 16], [16, 18]
]

class Pose_2d:
    num_kpts = 19
    kpt_names = ['neck', 'nose', 'hip', # (compute)
                 'l_sho', 'l_elb', 'l_wri', 'l_hip', 'l_knee', 'l_ank',
                 'r_sho', 'r_elb', 'r_wri', 'r_hip', 'r_knee', 'r_ank',
                 'l_eye', 'l_ear',
                 'r_ear', 'r_ear']
    sigmas = np.array([.26, .79, .79, .79, .72, .62, .79, .72, .62, 1.07, .87, .89, 1.07, .87, .89, .25, .25, .35, .35],
                      dtype=np.float32) / 10.0
    vars = (sigmas * 2) ** 2

    last_id = -1   # init id
    colors = ([255, 0, 0], [0, 0, 255], [0, 255, 0], [0, 255, 255], [255, 0, 255])

    # keypoints and all ave_confidence
    def __init__(self, pose_2d, confidence):
        super().__init__()
        self.keypoints = pose_2d
        self.confidence = confidence    # make avg scores of keypoints confidence as finally scores
        found_keypoints = np.zeros((np.count_nonzero(pose_2d[:, 0] != -1), 2), dtype=np.int32)
        found_kpt_id = 0
        for kpt_id in range(pose_2d.shape[0]):
            if pose_2d[kpt_id, 0] == -1:
                continue
            found_keypoints[found_kpt_id] = pose_2d[kpt_id]
            found_kpt_id += 1
        self.bbox = cv2.boundingRect(found_keypoints)
        self.id = id
        self.translation_filter = [OneEuroFilter(freq=80, beta=0.01),
                                   OneEuroFilter(freq=80, beta=0.01),
                                   OneEuroFilter(freq=80, beta=0.01)]
        self.state = None
    
    def filter(self, translation):
        filtered_translation = []
        for coordinate_id in range(3):
            filtered_translation.append(self.translation_filter[coordinate_id](translation[coordinate_id]))
        return filtered_translation

    def get_state(self, pose):
        pass

    def update_id(self, id=None):
        self.id = id
        if self.id is None:
            self.id = Pose_2d.last_id + 1
            Pose_2d.last_id += 1

colors = ([255, 0, 0], [0, 0, 255], [0, 255, 0], [0, 255, 255], [255, 0, 255])

def draw(pose, img, mode):
    if mode == 'reproject':
        point_color = colors[0]
        line_color = colors[1]
    if mode == 'norm':
        point_color = colors[1]
        line_color = colors[0]
    if mode == 'recom_project':
        point_color = colors[2]
        line_color = colors[3]

    assert pose.shape == (19, 3)

    for part_id in range(len(body_edges)):
        kpt_a_id = body_edges[part_id][0]
        a_conf = pose[kpt_a_id, 0]
        if a_conf != -1:  # Point exist
            joint_2d_a = pose[kpt_a_id]
            cv2.circle(img, (int(joint_2d_a[0]), int(joint_2d_a[1])), 3, point_color, 4)
        kpt_b_id = body_edges[part_id][1]
        b_conf = pose[kpt_b_id, 0]
        if b_conf != -1:
            joint_2d_b = pose[kpt_b_id]
            cv2.circle(img, (int(joint_2d_b[0]), int(joint_2d_b[1])), 3, point_color, 4)
        if a_conf != -1  and b_conf != -1:
            cv2.line(img, (int(joint_2d_a[0]), int(joint_2d_a[1])), (int(joint_2d_b[0]), int(joint_2d_b[1])), line_color, 4)

def transPose(pose_3d):
    pose = np.ones((3, 19), dtype=np.float32)
    for joint_id in range(pose_3d.shape[0]):
        pose[0, joint_id] = pose_3d[joint_id, 0]
        pose[1, joint_id] = pose_3d[joint_id, 1]
        pose[2, joint_id] = pose_3d[joint_id, 2]

    return pose

def returnPose(pose_3d):
    pose = np.ones((19, 3), dtype=np.float32)
    for joint_id in range(pose_3d.shape[1]):
        pose[joint_id, 0] = pose_3d[0, joint_id]
        pose[joint_id, 1] = pose_3d[1, joint_id]
        pose[joint_id, 2] = pose_3d[2, joint_id]

    return pose

def project_pose_from_camera(pose_3d, K, kd):
    #
    #         [x]   [f  0  cx  0]   [X_c]
    #   Z_c * [y] = [0  f  cy  0] * [Y_c]
    #         [1]   [0  0  1   0]   [Z_c]
    #                               [ 1 ]

    pose = np.zeros((len(pose_3d), 3), dtype=np.int32)
    for i in range(len(pose_3d)):
        pose[i][0] = int(pose_3d[i][0] * K[0, 0] / pose_3d[i][2] + K[0, 2])
        pose[i][1] = int(pose_3d[i][1] * K[1, 1] / pose_3d[i][2] + K[1, 2])
    return pose

def project_pose_from_world(pose_3d, R, t, K, kd):
    pose_3d = transPose(pose_3d)
    # pose_3d = pose_3d.reshape((-1, 3)).transpose()
    R = mat(R)
    x = np.asarray(R*pose_3d + t)
    # x = pose_3d[0:3, :] = np.dot(R, pose_3d[0:3, :] + t)
    x[0:2, :] = x[0:2, :]/x[2, :]
    r = x[0, :] * x[0, :] + x[1, :] * x[1, :]

    x[0, :] = x[0, :] * (1 + kd[0]*r + kd[1]*r*r + kd[4]*r*r*r) + 2*kd[2]*x[0, :]*x[1, :] + kd[3]*(r + 2*x[0, :]*x[0, :])
    x[1, :] = x[1, :] * (1 + kd[0]*r + kd[1]*r*r + kd[4]*r*r*r) + 2*kd[3]*x[0, :]*x[1, :] + kd[2]*(r + 2*x[1, :]*x[1, :])

    x[0, :] = K[0, 0]*x[0, :] + K[0, 1]*x[1, :] + K[0, 2]
    x[1, :] = K[1, 0]*x[0, :] + K[1, 1]*x[1, :] + K[1, 2]

    pose = returnPose(x)
    return pose

# camera to world
def rotate_poses(pose_3d, R, t):
    # [X_c]       [X_w]
    # [Y_c]  =  R [Y_w]  +  t
    # [Z_c]       [Z_w]

    R_inv = np.linalg.inv(R)
    pose_3d = pose_3d.reshape((-1, 3)).transpose()
    pose_3d[0:3, :] = np.dot(R_inv, pose_3d[0:3, :] - t)
    pose_3d = pose_3d.transpose()

    return pose_3d

def compute_pose_3d(pose_2d, pose_depth, K):
    # [X, Y, Z]^T = Z * K^(-1) * [x, y, 1]^T
    scores = pose_2d[:, 2]
    joint_camera = np.zeros((len(pose_2d), 4), dtype=np.float32)
    joint_camera[:, 3] = scores
    pose_2d[:, 2] = 1
    K_inv = np.linalg.inv(K)
    K_inv = mat(K_inv)

    # joint_world = np.zeros((len(pose_2d), 4), dtype=np.float32)
    for joint_id in range(len(pose_2d)):
        joint = pose_2d[joint_id]
        joint = mat(joint)
        joint = joint.transpose()
        joint_cam = pose_depth[joint_id] * K_inv * joint

        joint_camera[joint_id, :3] = joint_cam.transpose().A

    return joint_camera


class Pose_3d:
    num_kpts = 19
    kpt_names = ['neck', 'nose', 'hip',  # (compute)
                 'l_sho', 'l_elb', 'l_wri', 'l_hip', 'l_knee', 'l_ank',
                 'r_sho', 'r_elb', 'r_wri', 'r_hip', 'r_knee', 'r_ank',
                 'l_eye', 'l_ear',
                 'r_ear', 'r_ear']

    sigmas = np.array([.26, .79, .79, .79, .72, .62, .79, .72, .62, 1.07, .87, .89, 1.07, .87, .89, .25, .25, .35, .35],
                      dtype=np.float32) / 10.0
    vars = (sigmas * 2) ** 2

    last_id = -1

    # world means the point is world point
    def __init__(self, keypoints_3d, id, cam_mode, world):
        # Marker
        # self.marker = MarkerGenerator()
        self.marker = Marker()   #add
        if world:
            self.marker.header.frame_id = "marker_0"
        else:
            self.marker.header.frame_id = "camera_base_1"
        self.marker.ns = "human"
        # self.marker.type = Marker.LINE_STRIP
        self.marker.type = Marker.LINE_LIST  # add
        self.marker.scale = [0.01, 0, 0]
        self.marker.color = [1, 0, 0, 1]


        self._keypoints_3d = keypoints_3d
        self._id = id
        self._R = Camera(cam_mode).R
        self._t = Camera(cam_mode).t
        self._K = Camera(cam_mode).K
        self._D = Camera(cam_mode).D

        if world:
            self._pose_2d = project_pose_from_world(self._keypoints_3d, self._R, self._t, self._K, self._D)
        else:
            self._pose_2d = project_pose_from_camera(self._keypoints_3d, self._K, self._D)

        found_keypoints = np.zeros((np.count_nonzero(self._pose_2d[:, 0] != -1), 2), dtype=np.int32)
        found_kpt_id = 0
        for kpt_id in range(self._pose_2d.shape[0]):
            if self._pose_2d[kpt_id, 0] == -1:
                continue
            found_keypoints[found_kpt_id] = self._pose_2d[kpt_id][:2]
            found_kpt_id += 1

        self._bbox = cv2.boundingRect(found_keypoints)
        self.translation_filter = [OneEuroFilter(freq=80, beta=0.01),
                                   OneEuroFilter(freq=80, beta=0.01),
                                   OneEuroFilter(freq=80, beta=0.01)]


    def filter(self, translation):
        filtered_translation = []
  
        for coordinate_id in range(3):
            filtered_translation.append(self.translation_filter[coordinate_id](translation[coordinate_id]))
        return filtered_translation

    def update_id(self, id=None):
        self._id = id
        if self._id is None:
            self._id = Pose_3d.last_id + 1
            Pose_3d.last_id += 1

    # def show_id(self, img):
    #     cv2.putText(img, 'id: {}'.format(self._id), (int(self._bbox[0]), int(self._bbox[1] - 100)),
    #                         cv2.FONT_HERSHEY_COMPLEX, 1, [255, 0, 0])

    def draw_pose(self, img):
        draw(self._pose_2d, img, 'recom_project')
        cv2.putText(img, 'id: {}'.format(self._id), (int(self._bbox[0]), int(self._bbox[1] - 100)),
                    cv2.FONT_HERSHEY_COMPLEX, 1, [255, 0, 0])


    def publish_pose_3d(self, markerPub):
        for i in range(len(body_edges)):
            m = self.marker.marker(points=[self._keypoints_3d[body_edges[i][0]]/100, self._keypoints_3d[body_edges[i][1]]/100])
            markerPub.publish(m)

    def set_pose_3d(self):
        # print(self._id, self._keypoints_3d)
        markerarray = MarkerArray()
        for i in range(len(body_edges)):
            self.marker.points = [self._keypoints_3d[body_edges[i][0]]/100, self._keypoints_3d[body_edges[i][1]]/100]
            markerarray.markers.append(self.marker)
        return markerarray

#add
class LINE_MARKER():
    def __init__(self):
        self.marker = Marker()
        self.marker.header.frame_id = 'camera_base_1'
        # 设置性质
        self.marker.type = self.marker.LINE_LIST
        self.marker.action = self.marker.ADD
        self.marker.ns = 'lines'
        self.marker.scale.x = 0.03
        self.marker.scale.y = 0
        self.marker.scale.z = 0
        self.marker.color.r = 1.0
        self.marker.color.a = 1.0
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0

        # marker position
        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = 0.0
        self.marker.id = 0

    # 保存点
    def store_points(self, keypoints_3D):
        self.marker.points = []
        for i in range(len(keypoints_3D)):
            for j in range(len(body_edges)):
                p1 = Point()
                p2 = Point()
                # 按照顺序连接
                k1 = body_edges[j][0]
                k2 = body_edges[j][1]
                # 点
                p1.x = keypoints_3D[i][k1][0]/100
                p1.y = keypoints_3D[i][k1][1]/100
                p1.z = keypoints_3D[i][k1][2]/100
                # print(p1)
                # 添加点
                self.marker.points.append(p1)
                p2.x = keypoints_3D[i][k2][0]/100
                p2.y = keypoints_3D[i][k2][1]/100
                p2.z = keypoints_3D[i][k2][2]/100
                self.marker.points.append(p2)

        return self.marker



def get_similarity(a, b, threshold=0.5):
    num_similar_kpt = 0
    for kpt_id in range(Pose_2d.num_kpts):
        if a.keypoints[kpt_id, 0] != -1 and b.keypoints[kpt_id, 0] != -1:
            distance = np.sum((a.keypoints[kpt_id] - b.keypoints[kpt_id]) ** 2)
            area = max(a.bbox[2] * a.bbox[3], b.bbox[2] * b.bbox[3])
            similarity = np.exp(-distance / (2 * (area + np.spacing(1)) * Pose_2d.vars[kpt_id]))
            if similarity > threshold:
                num_similar_kpt += 1
    return num_similar_kpt


def propagate_ids(previous_poses, current_poses, threshold=3):
    """Propagate poses ids from previous frame results. Id is propagated,
    if there are at least `threshold` similar keypoints between pose from previous frame and current.

    :param previous_poses: poses from previous frame with ids
    :param current_poses: poses from current frame to assign ids
    :param threshold: minimal number of similar keypoints between poses
    :return: None
    """
    current_poses_sorted_ids = list(range(len(current_poses)))
    current_poses_sorted_ids = sorted(
        current_poses_sorted_ids, key=lambda pose_id: current_poses[pose_id].confidence, reverse=True)  # match confident poses first
    mask = np.ones(len(previous_poses), dtype=np.int32)
    for current_pose_id in current_poses_sorted_ids:
        best_matched_id = None
        best_matched_pose_id = None
        best_matched_iou = 0
        for previous_pose_id in range(len(previous_poses)):
            if not mask[previous_pose_id]:
                continue
            iou = get_similarity(current_poses[current_pose_id], previous_poses[previous_pose_id])
            if iou > best_matched_iou:
                best_matched_iou = iou
                best_matched_pose_id = previous_poses[previous_pose_id].id
                best_matched_id = previous_pose_id
        if best_matched_iou >= threshold:
            mask[best_matched_id] = 0
        else:  # pose not similar to any previous
            best_matched_pose_id = None
        current_poses[current_pose_id].update_id(best_matched_pose_id)
        if best_matched_pose_id is not None:
            current_poses[current_pose_id].translation_filter = previous_poses[best_matched_id].translation_filter