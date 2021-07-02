import numpy as np
import cv2

from modules.pose import Pose_2d, propagate_ids
from PIL import Image
import matplotlib.pyplot as plt
import matplotlib.image as im

fast = False

try:
   from pose_extractor import extract_poses
   fast = True

except:
    print("Please use fast 2d pose extractor")
    from modules.slow_extractor import extract_poses

from IPython import embed

# miss neck id=2
map_id_panoptic = [1, 0, 9, 10, 11, 3, 4, 5, 12, 13, 14, 6, 7, 8, 15, 16, 17, 18]

all_limbs = {
    'head': [[1, 17, 18], [1, 15, 16]],
    'torso': [[1, 0, 2]],
    'neck': 0,
    'f_limbs': [[3, 4, 5], [9, 10, 11], [6, 7, 8], [12, 13, 14]]
}
#joint pair
jointPair = [
    [0, 1], [0, 2], [0, 9],
    [9, 10], [10, 11],
    [0, 3], [3, 4], [4, 5],
    [2, 12], [12, 13], [13, 14],
    [2, 6], [6, 7], [7, 8]
]

#statistic bone length
bone_length = [26.42178982, 48.36980909, 14.88291009, 31.28002332, 23.915707, 14.97674918,
               31.28002549, 23.91570732, 12.4644364,  48.26604433, 39.03553194,
               12.4644364, 48.19076948, 39.03553252]

AVG_PERSON_HIGHT = 180

def adaptive_bone_constrains(pose_2d, fx, depth):
    assert pose_2d.size == 59
    image_width = 1920
    error = 0.0
    ratio = fx / (image_width * depth)
    for i in range(len(jointPair)):
        joint_a = pose_2d[3*jointPair[i][0]:3*(jointPair[i][0]+1) / 100]
        joint_b = pose_2d[3*jointPair[i][1]:3*(jointPair[i][1]+1) / 100]
        error += pow((joint_a[:2] - joint_b[:2]), 2)
    print(error)
    embed()


def get_root_relative_pose(inference_result, fx):
    location_maps, heatmaps, pafs = inference_result
    #
    # location_maps[0] = (location_maps[0] - np.min(location_maps[0]))/(np.max(location_maps[0]) - np.min(location_maps[0]))
    # location_maps[0] *= 255
    # img = Image.fromarray(location_maps[0])
    # if img.mode == 'F':
    #     img = img.convert('RGB')
    # img.save("map_np.jpg")
    #
    # embed()

    # dst = cv2.resize(location_maps[2], (1920, 1080))
    # cv2.imwrite('map_z.jpg', dst)
    # cv2.imshow('heat', dst)
    # cv2.waitKey(0)

    upsample_ratio = 4

    if (fast):
        # fast extract pose with c++
        poses_2d = []
        num_keypoints = 18
        num_kpt_panoptic = 19
        pose_enteries = extract_poses(heatmaps[0:-1], pafs, upsample_ratio)[0]

        pose_enteries[:, 0:-1:3] /= upsample_ratio
        pose_enteries[:, 1:-1:3] /= upsample_ratio

        for pose_id in range(pose_enteries.shape[0]):
            if pose_enteries[pose_id, 5] == -1:  # if neck not found, skip pose
                continue

            correspondence = 0.0
            count = 0
            for kpt in range(num_keypoints):
                if pose_enteries[pose_id, kpt*3 + 2] != -1:
                    correspondence += pose_enteries[pose_id, kpt*3 + 2]
                    count += 1
            correspondence /= count

            if(correspondence > 0.6):
                # +1 for neck depth
                pose_2d = np.ones((num_kpt_panoptic * 3 + 2), dtype=np.float32) * -1    # x, y, p
                for kpt_id in range(num_keypoints):
                    if pose_enteries[pose_id, kpt_id * 3] != -1:
                        x_2d, y_2d = pose_enteries[pose_id, kpt_id * 3: kpt_id * 3 + 2]
                        score = pose_enteries[pose_id, kpt_id * 3 + 2]

                        # redefine the joint id from coco dataset to panoptic dataset
                        pose_2d[map_id_panoptic[kpt_id] * 3] = x_2d
                        pose_2d[map_id_panoptic[kpt_id] * 3 + 1] = y_2d
                        pose_2d[map_id_panoptic[kpt_id] * 3 + 2] = score
            #    pose_2d = Pose(pose_2d, pose_enter)
                poses_2d.append(pose_2d)
            else:
                continue

        # for pose_id in range(len(poses_2d)):
        #     pose_2d = poses_2d[pose_id]
        #     ave_score = 0.0
        #     for i in range(1, len(pose_2d)//3):
        #         ave_score += pose_2d[2*i]
        #     embed()
            #if adaptive_bone_constrains(poses_2d[pose_id], fx, poses_2d[pose_id][-2]):
                #poses_2d[pose_id][-1] = 10
            #else:
                #del poses_2d[pose_id]

        # re-ranking all poses by neck root depth
        # and read out neck location
        for pose_id in range(len(poses_2d)):
            if poses_2d[pose_id][2] == -1:
                continue
            neck_2d = poses_2d[pose_id][:2].astype(int)
            map_3d = location_maps[0:3]
            poses_2d[pose_id][-2] = map_3d[2, neck_2d[1], neck_2d[0]] * AVG_PERSON_HIGHT


        # re-ranking 2d poses
        poses_2d = [pose_id for pose_id in poses_2d if pose_id[2] != -1]
        poses_2d.sort(key=lambda x:(x[-1]), reverse=True)

        # for pose_id in range(pose_enteries.shape[0]):
        #     if pose_enteries[pose_id, 5] == -1:  # if neck not found, skip pose
        #         continue
        #     # there is a problem about keypoints position. It is a negative number.
        #
        #     pose_2d = np.ones((num_keypoints, 3), dtype=np.float32) * -1    # x, y, p
        #     for kpt_id in range(num_keypoints):
        #         if pose_enteries[pose_id, kpt_id * 2] > 0:
        #             x_2d, y_2d = pose_enteries[pose_id, kpt_id * 3: kpt_id * 3 + 2]
        #             score = pose_enteries[pose_id, kpt_id * 3 + 2]
        #             pose_2d[kpt_id, 0] = x_2d
        #             pose_2d[kpt_id, 1] = y_2d
        #             pose_2d[kpt_id, 2] = score
        #     # pose_2d = Pose(pose_2d, pose_enter)
        #     poses_2d.append(pose_2d)

    # estimate the 3D pose p=p_i {m, i=1} for each of the m persons in the image.
    # P_i(R(3xn)) describes the 3D locations of the n=17 body joints of person i.
    # The body joint locations are expressed relative to the parent joints.
    # first, decompose the body into pelvis, neck, head, and a set of limbs L.
        # L={shoulder, elbow, wrist}, {hip, knee, ankle}

    # the 3D locations of the joints are encoded in the ORPM denoted by M={Mj}{n, j=1}, where Mj(R(WxHx3)).
    # some special cases should be cared:
        # 1. Locations coincide: Mj should contains information about the person closer to the camera at the overlapping locations.
        # however, the occluded person can be obtained at other available read-out locations.
        # 2.

    # Read-out process
        # 1. Define extremity joints(???): the wrists, the ankle, the head.
        # 2. Start reading the full base pose at the neck location. If the neck is invalid then the full pose is read at
            # the pelvis instead. If both of these joints are invalid, we consider this person as not visible in the
            # scene and we do not predict the person's pose.
        # 3. Full pose read at the pelvis and neck tend to be closer to the average pose in the training data.
            # So, for each limb, we continue by reading out the limb pose(can be read at any of the limb's 2D
            # joint location) at extremity joints.
        # 4. If the extremity joint is valid, the limb pose replaces the elements of the base pose; if it is invalid,
            # walk up the kinematic chain and check other joints of this limb; If all joint of the limb are invalid, the
            # base pose cannot refined.

    # Valid 2D Pose
        # for 2d joint Pij = (u, v)ij
        # 1. it is un-occluded and confidence higher than the threshold
        # 2. it is sufficiently far away from all read-out locations of joint j of other individuals.
        keypoint_threshold = 0.1
        poses_3d = np.ones((len(poses_2d), num_kpt_panoptic, 4), dtype=np.float32) * -1
        for pose_id in range(len(poses_3d)):
            if poses_2d[pose_id][2] > keypoint_threshold:   # person is detected
                neck_2d = poses_2d[pose_id][:2].astype(int)    # read all neck location

                # 1. redundancy(1), read all pose at neck location
                for kpt_id in range(num_kpt_panoptic):
                    map_3d = location_maps[kpt_id * 3 : (kpt_id + 1) * 3]
                    poses_3d[pose_id][kpt_id][0] = map_3d[0, neck_2d[1], neck_2d[0]] * AVG_PERSON_HIGHT
                    poses_3d[pose_id][kpt_id][1] = map_3d[1, neck_2d[1], neck_2d[0]] * AVG_PERSON_HIGHT
                    poses_3d[pose_id][kpt_id][2] = map_3d[2, neck_2d[1], neck_2d[0]] * AVG_PERSON_HIGHT
                    poses_3d[pose_id][kpt_id][3] = poses_2d[pose_id][2]
                # 2. redundancy(2)
                # refined keypoints of four limbs
                for limb in all_limbs['f_limbs']:
                    for limb_id in limb:
                        if poses_2d[pose_id][limb_id * 3 + 2] > keypoint_threshold:
                            for kpt_id in limb:
                                kpt_from_2d = poses_2d[pose_id][limb_id * 3 : limb_id * 3 + 2].astype(int)
                                map_3d = location_maps[kpt_id * 3 : (kpt_id + 1) * 3]
                                poses_3d[pose_id][kpt_id][0] = map_3d[0, kpt_from_2d[1], kpt_from_2d[0]] * AVG_PERSON_HIGHT
                                poses_3d[pose_id][kpt_id][1] = map_3d[1, kpt_from_2d[1], kpt_from_2d[0]] * AVG_PERSON_HIGHT
                                poses_3d[pose_id][kpt_id][2] = map_3d[2, kpt_from_2d[1], kpt_from_2d[0]] * AVG_PERSON_HIGHT
                            break

                # 3. redundancy(3)
                # refined keypoints of torso
                for limb in all_limbs['torso']:
                    for limb_id in limb:
                        if poses_2d[pose_id][limb_id * 3 + 2] > keypoint_threshold:
                            for kpt_id in limb:
                                kpt_from_2d = poses_2d[pose_id][limb_id * 3: limb_id * 3 + 2].astype(int)
                                map_3d = location_maps[kpt_id * 3: (kpt_id + 1) * 3]
                                poses_3d[pose_id][kpt_id][0] = map_3d[0, kpt_from_2d[1], kpt_from_2d[0]] * AVG_PERSON_HIGHT
                                poses_3d[pose_id][kpt_id][1] = map_3d[1, kpt_from_2d[1], kpt_from_2d[0]] * AVG_PERSON_HIGHT
                                poses_3d[pose_id][kpt_id][2] = map_3d[2, kpt_from_2d[1], kpt_from_2d[0]] * AVG_PERSON_HIGHT
                            break

                # 4. redundancy(4)
                # refined keypoints of head
                for limb in all_limbs['head']:
                    for limb_id in limb:
                        if poses_2d[pose_id][limb_id * 3 + 2] > keypoint_threshold:
                            for kpt_id in limb:
                                kpt_from_2d = poses_2d[pose_id][limb_id * 3: limb_id * 3 + 2].astype(int)
                                map_3d = location_maps[kpt_id * 3: (kpt_id + 1) * 3]
                                poses_3d[pose_id][kpt_id][0] = map_3d[0, kpt_from_2d[1], kpt_from_2d[0]] * AVG_PERSON_HIGHT
                                poses_3d[pose_id][kpt_id][1] = map_3d[1, kpt_from_2d[1], kpt_from_2d[0]] * AVG_PERSON_HIGHT
                                poses_3d[pose_id][kpt_id][2] = map_3d[2, kpt_from_2d[1], kpt_from_2d[0]] * AVG_PERSON_HIGHT
                            break
        return poses_3d, np.array(poses_2d), location_maps.shape

    else:
        found_pose = extract_poses(heatmaps, pafs, upsample_ratio, stride, scale, pad)
        return found_pose

previous_poses_2d = []

def parse_pose(inference_result, stride, scale, fx, is_video=True):
    poses_3d, poses_2d, map_shape = get_root_relative_pose(inference_result, fx)

    # get Pose_2d
    poses_2d_scale = []
    for pose_2d in poses_2d:
        num_kpt = (pose_2d.shape[0] - 1) // 3
        pose_2d_scale = np.ones((num_kpt, 3), dtype=np.float32) * -1
        for kpt_id in range(num_kpt):
            if pose_2d[kpt_id * 3 + 2] != -1:
                pose_2d_scale[kpt_id, 0] = int(pose_2d[kpt_id * 3] * stride / scale)
                pose_2d_scale[kpt_id, 1] = int(pose_2d[kpt_id * 3 + 1] * stride / scale)
                pose_2d_scale[kpt_id, 2] = pose_2d[kpt_id * 3 + 2]
        if pose_2d_scale[6][2] != -1 and pose_2d_scale[12][2] != -1:
            pose_2d_scale[2] = (pose_2d_scale[6] + pose_2d_scale[12]) / 2
        poses_2d_scale.append(pose_2d_scale)

    global previous_poses_2d
    if is_video:  # track poses ids
        current_poses_2d = []
        for pose_id in range(len(poses_2d_scale)):
            pose_keypoints = np.ones((Pose_2d.num_kpts, 2), dtype=np.int32) * -1
            confidence = 0.0
            count = 0
            for kpt_id in range(Pose_2d.num_kpts):
                if poses_2d_scale[pose_id][kpt_id, 2] != -1.0:  # keypoint was found
                    pose_keypoints[kpt_id, 0] = int(poses_2d_scale[pose_id][kpt_id, 0])
                    pose_keypoints[kpt_id, 1] = int(poses_2d_scale[pose_id][kpt_id, 1])
                    confidence += poses_2d_scale[pose_id][kpt_id, 2]
                    count += 1
            pose_2d = Pose_2d(pose_keypoints, confidence/count)
            current_poses_2d.append(pose_2d)
        propagate_ids(previous_poses_2d, current_poses_2d)
        previous_poses_2d = current_poses_2d

    translated_poses_3d = []
    joint_depth=[]
    for pose_id in range(len(poses_3d)):
        pose_3d = poses_3d[pose_id].reshape((-1, 4)).transpose()
        pose_2d = poses_2d[pose_id][:-2].reshape((-1, 3)).transpose()
        num_valid = np.count_nonzero(pose_2d[2] != -1)
        pose_3d_valid = np.zeros((3, num_valid), dtype=np.float32)
        pose_2d_valid = np.zeros((2, num_valid), dtype=np.float32)
        valid_id = 0
        for kpt_id in range(pose_3d.shape[1]):
            if pose_2d[2, kpt_id] == -1:
                continue
            pose_3d_valid[:, valid_id] = pose_3d[0:3, kpt_id]
            pose_2d_valid[:, valid_id] = pose_2d[0:2, kpt_id]
            valid_id += 1

        pose_2d_valid[0] = pose_2d_valid[0] - map_shape[2]/2
        pose_2d_valid[1] = pose_2d_valid[1] - map_shape[1]/2
        mean_3d = np.expand_dims(pose_3d_valid.mean(axis=1), axis=1)
        mean_2d = np.expand_dims(pose_2d_valid.mean(axis=1), axis=1)

        numerator = np.trace(np.dot((pose_3d_valid[:2, :] - mean_3d[:2, :]).transpose(),
                                    pose_3d_valid[:2, :] - mean_3d[:2, :])).sum()
        numerator = np.sqrt(numerator)
        denominator = np.sqrt(np.trace(np.dot((pose_2d_valid[:2, :] - mean_2d[:2, :]).transpose(),
                                              pose_2d_valid[:2, :] - mean_2d[:2, :])).sum())
        mean_2d = np.array([mean_2d[0, 0], mean_2d[1, 0], fx * scale / stride])
        mean_3d = np.array([mean_3d[0, 0], mean_3d[1, 0], 0])
        translation = numerator / denominator * mean_2d - mean_3d

        if is_video:
            translation = current_poses_2d[pose_id].filter(translation)
        for kpt_id in range(pose_3d.shape[1]):
            pose_3d[0, kpt_id] = pose_3d[0, kpt_id] + translation[0]
            pose_3d[1, kpt_id] = pose_3d[1, kpt_id] + translation[1]
            pose_3d[2, kpt_id] = pose_3d[2, kpt_id] + translation[2]
        translated_poses_3d.append(pose_3d.transpose().reshape(-1))
        joint_depth.append(pose_3d[2])

    return np.array(translated_poses_3d), poses_2d_scale, np.array(joint_depth)