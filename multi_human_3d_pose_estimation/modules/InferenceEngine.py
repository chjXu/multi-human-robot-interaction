# This file is to infer the output of model
import numpy as np
import torch
import cv2
import math

from models.with_mobilenet import PoseEstimationWithMobileNet
from modules.load_state import load_state
from IPython import embed
from torchstat import stat

from thop import profile
from torch import nn


def modelsize(model, input, type_size=4):
    param = sum([np.prod(list(p.size())) for p in model.parameters()])
    print("Model {} : params: {:4f}M".format(model._get_name(), param*4/1000/1000))

    input_ = input.clone()
    input_.requires_grad_(requires_grad=False)

    mods = list(model.modules())
    out_sizes=[]

    for i in range(1, len(mods)):
        m = mods[i]
        print(i)
        if isinstance(m, nn.ReLU):
            if m.inplace:
                continue
        out = m(input_)
        print(out.shape)
        out_sizes.append(np.array(out.size()))
        input_ = out
    
    total_nums = 0
    for i in range(len(out_sizes)):
        s = out_sizes[i]
        nums = np.prod(np.array(s))
        total_nums += nums
    print("Model {} : intermedite variables: {:3f}M".format(model._get_name(), total_nums*4/1000/1000))

class InferenceEngine:
    def __init__(self, checkpoint_path, device, gpu_id, stride=8, img_mean=np.array([128, 128, 128], dtype=np.float32), pad_value=(0, 0, 0),
                 img_scale=np.float32(1/255)):
        self.img_mean = img_mean
        self.img_scale = img_scale
        self.stride = stride
        self.pad_value = pad_value
        self.device = device
        self.gpu_id = gpu_id

        if device != 'CPU':
            if torch.cuda.is_available():
                print('cuda:' + str(self.gpu_id))
                self.device = torch.device('cuda:' + str(self.gpu_id))
            else:
                #self.device = 'cpu'
                print("No GPU found, infer by CPU")

        # load net and checkpoint
        net = PoseEstimationWithMobileNet()
        # print(net)
        checkpoint = torch.load(checkpoint_path, map_location='cpu')
        load_state(net, checkpoint)

        net = net.to(self.device)
        net.eval()
        self.net = net

    # the process of image
    def infer(self, img):
        # height, width, _ = img.shape
        # scale = net_input_height_size / height
        #
        # scaled_img = cv2.resize(img, (0, 0), fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
        normal_img = InferenceEngine._normalize(img, self.img_mean, self.img_scale)
        # min_dims = [net_input_height_size, max(scaled_img.shape[1], net_input_height_size)]
        # padded_img, pad = InferenceEngine._pad_width(scaled_img, self.stride, self.pad_value, min_dims)
        #
        data = torch.from_numpy(normal_img).permute(2, 0, 1).unsqueeze(0).to(self.device)
        #print(data.shape)
        #modelsize(self.net, data)
        #stat(self.net, input_size=(1, 3, 256, 448))
        
        
        # location is the output of pose3D network, that has a fixed number of outputs regardless of the number of people
        # in the scene, while enabling pose read-outs for strongly occluded people.

        # represent the body by decomposing it into torso, four limbs, abd head.
        # multiple levels of redundancy:
        #   1. read-out the complete base pose P(3xn) at one of the torso joint locations(neck or pelvis)
        #   2. the base pose can be refined by reading out the head and limbs poses where 2D detections are available.
        #   3. complete limb pose can be read out at any 2D joint location of that limb.
        
        # the 3D locations of the joints are encoded in the ORPM denoted by M={Mj}{n, j=1}, where Mj(R(WxHx3)).
        location_maps, keypoints_maps, paf_maps = self.net(data)
        # stage_output is the out of openpose,
        # where stage_output[-1] is paf and stage_out[-2] is heatmap
        # embed()
        return (location_maps[-1].squeeze().cpu().data.numpy(),
                keypoints_maps[-1].squeeze().cpu().data.numpy(), paf_maps[-1].squeeze().data.cpu().numpy())

    @staticmethod
    def _normalize(img, img_mean, img_scale):
        # img = np.array(img, dtype=np.float32)
        # img = (img - img_mean) * img_scale
        img = (img.astype(np.float32) - img_mean) * img_scale
        return img

    @staticmethod
    def _pad_width(img, stride, pad_value, min_dims):
        h, w, _ = img.shape
        h = min(min_dims[0], h)
        min_dims[0] = math.ceil(min_dims[0] / float(stride)) * stride
        min_dims[1] = max(min_dims[1], w)
        min_dims[1] = math.ceil(min_dims[1] / float(stride)) * stride
        pad = []
        pad.append(int(math.floor((min_dims[0] - h) / 2.0)))
        pad.append(int(math.floor((min_dims[1] - w) / 2.0)))
        pad.append(int(min_dims[0] - h - pad[0]))
        pad.append(int(min_dims[1] - w - pad[1]))
        padded_img = cv2.copyMakeBorder(img, pad[0], pad[2], pad[1], pad[3],
                                        cv2.BORDER_CONSTANT, value=pad_value)
        return padded_img, pad
