import numpy as np

def coco17topanoptic(coco_pose):
    #   kpt_names = ['nose', 'neck', 'r_sho', 'r_elb', 'r_wri', 'l_sho', 'l_elb', 'l_wri',
    #             'r_hip', 'r_knee', 'r_ank', 'l_hip', 'l_knee', 'l_ank',
    #             'r_eye', 'l_eye',
    #             'r_ear', 'l_ear']

    #   kpt_panoptic = ['neck', 'nose', 'hip', 'l_sho', 'l_elb', 'l_wri', 'l_hip', 'l_knee',
    #             'l_ankle', 'r_sho', 'r_elb', 'r_wri', 'r_hip', 'r_knee', 'r_ankle',
    #             'l_eye', 'l_ear',
    #             'r_eye', 'r_ear']

    panoptic = np.zeros((19, coco_pose.shape[1]))
    index_array = np.array([1, 15, 17, 16, 18, 3, 9, 4, 10, 5, 11, 6, 12, 7, 13, 8, 14])
    panoptic[index_array] = coco_pose
    panoptic[0] = (coco_pose[5] + coco_pose[6]) / 2
    panoptic[2] = (coco_pose[11] + coco_pose[12]) / 2
    panoptic[-4:] = coco_pose[0]
    return panoptic