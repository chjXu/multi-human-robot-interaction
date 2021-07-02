# from keras.models import load_model
# from modules.pose import Pose_2d
import sys
sys.path.append("/home/xuchengjun/multi_human/action")
import numpy as np
import ast
from IPython import embed
import torch
from train import Net
import time

    # T_pose = 0  #识别操作人员
    # stand = 1   #
    # sit = 2
    # handing = 3
    # hit = 4

action_dict = ["T-pose", "stand", "sit", "handing", "hit"]

# class ActionClassifier(object):
#     def __init__(self, model_path):
#         self.dnn_model = load_model(model_path)
#         self.action_dict = ["kick", "punch", "squat", "stand", "wave"]

#     def predict(self, pose):
#         # Preprocess data
#         tmp = pose_normalization(pose)
#         skeleton_input = np.array(tmp).reshape(-1, len(tmp))

#         # Predicted label: int & string
#         predicted_idx = np.argmax(self.dnn_model.predict(skeleton_input))
#         prediced_label = self.action_dict[predicted_idx]

#         pose.state = prediced_label

# def load_action_premodel(model):
#     return load_model(model)

# def action_recognition(pose, pretrained_model):
#     joints = pose.keypoints

#     tmp = pose_normalization(joints)
#     skeleton_input = np.array(tmp).reshape(-1, len(tmp))

#     # Predicted label: int & string
#     predicted_idx = np.argmax(pretrained_model.predict(skeleton_input))
#     prediced_label = self.action_dict[predicted_idx]

#     # if len(joints)>0:
#     #     joints_norm_single_person = np.array(joints).reshape(-1, 36)
#     #     pred = np.argmax(pretrained_model.predict(joints_norm_single_person))
#     #     init_label = Actions(pred).name

#     pose.state = prediced_label

# def framewise_recognize(pose, pretrained_model):
#     frame, joints, bboxs, xcenter = pose[0], pose[1], pose[2], pose[3]
#     joints_norm_per_frame = np.array(pose[-1])

#     if bboxs:
#         bboxs = np.array(bboxs)
#         features = encoder(frame, bboxs)

def load_model(mode_path):
    state_dict = torch.load(mode_path)
    net = Net()
    net.load_state_dict(state_dict)
    return net

def predict(data,net):
    data = torch.Tensor(data)
    out = net.forward(data)
    label_num = out.argmax()
    return label_num

if __name__ == "__main__":
    #load the model
    net = load_model('action.pkl')
    # print(net)
    total = 0
    right = 0
    total_time = 0.0
    i = 0
    with open('../data/t_test.txt') as file:
        start_time = time.time()
        while i<4:
            # start_time = time.time()
            lines = file.readline()
            lines = ast.literal_eval(lines)
            if not lines:
                break
            total += 1
            # data = [0.78, 0.034, 0.584, 0.568, 0.951, 0.184, 1.0, 0.391, 0.974, 0.552, 0.754, 0.586, 0.731, 0.828, 0.659, 1.0, 0.364, 0.15, 0.0, 0.264, 0.168, 0.218, 0.413, 0.552, 0.364, 0.793, 0.364, 0.954, 0.731, 0.0, 0.85, 0.0, 0.61, 0.0, 0.925, 0.034]
            action = predict(torch.Tensor(lines),net)
            # end_time = time.time()
            if action == "T-pose":
                right +=  1
            i += 1
        end_time = time.time()
        total_time =  end_time  - start_time
    print("acc:",right/total)
    print("use time:{}".format(total_time))
    # data = [0.78, 0.034, 0.584, 0.568, 0.951, 0.184, 1.0, 0.391, 0.974, 0.552, 0.754, 0.586, 0.731, 0.828, 0.659, 1.0, 0.364, 0.15, 0.0, 0.264, 0.168, 0.218, 0.413, 0.552, 0.364, 0.793, 0.364, 0.954, 0.731, 0.0, 0.85, 0.0, 0.61, 0.0, 0.925, 0.034]    
    # action = predict(torch.Tensor(data),net)
    # print(action)
    

