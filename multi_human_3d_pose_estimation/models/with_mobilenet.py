import torch
from torch import nn
from modules.conv import conv, conv_dw, conv_dw_no_bn
from IPython import embed

#   Cpm(Convolutional Pose Machines)
#   Extract keypoints and score
class Cpm(nn.Module):
    def __init__(self, in_channels, out_channels):
        super().__init__()
        self.align = conv(in_channels, out_channels, kernel_size=1, padding=0, bn=False)
        self.trunk = nn.Sequential(
            conv_dw_no_bn(out_channels, out_channels),
            conv_dw_no_bn(out_channels, out_channels),
            conv_dw_no_bn(out_channels, out_channels)
        )
        self.conv = conv(out_channels, out_channels, bn=False)

    def forward(self, x):
        x = self.align(x)
        x = self.conv(x + self.trunk(x))
        return x


class InitialStage(nn.Module):
    def __init__(self, num_channels, num_heatmaps, num_pafs):
        super().__init__()
        self.trunk = nn.Sequential(
            conv(num_channels, num_channels, bn=False),
            conv(num_channels, num_channels, bn=False),
            conv(num_channels, num_channels, bn=False)
        )
        self.heatmaps = nn.Sequential(
            conv(num_channels, 512, kernel_size=1, padding=0, bn=False),
            conv(512, num_heatmaps, kernel_size=1, padding=0, bn=False, relu=False)
        )
        self.pafs = nn.Sequential(
            conv(num_channels, 512, kernel_size=1, padding=0, bn=False),
            conv(512, num_pafs, kernel_size=1, padding=0, bn=False, relu=False)
        )

    def forward(self, x):
        trunk_features = self.trunk(x)
        heatmaps = self.heatmaps(trunk_features)
        pafs = self.pafs(trunk_features)
        return [heatmaps, pafs]  #3.29 []


class RefinementStageBlock(nn.Module):
    def __init__(self, in_channels, out_channels):
        super().__init__()
        self.initial = conv(in_channels, out_channels, kernel_size=1, padding=0, bn=False)
        self.trunk = nn.Sequential(
            conv(out_channels, out_channels),
            conv(out_channels, out_channels, dilation=2, padding=2)
        )

    def forward(self, x):
        initial_features = self.initial(x)
        trunk_features = self.trunk(initial_features)
        return initial_features + trunk_features


class RefinementStage(nn.Module):
    def __init__(self, in_channels, out_channels, num_heatmaps, num_pafs):
        super().__init__()
        self.trunk = nn.Sequential(
            RefinementStageBlock(in_channels, out_channels),
            RefinementStageBlock(out_channels, out_channels),
            RefinementStageBlock(out_channels, out_channels),
            RefinementStageBlock(out_channels, out_channels),
            RefinementStageBlock(out_channels, out_channels)
        )
        self.heatmaps = nn.Sequential(
            conv(out_channels, out_channels, kernel_size=1, padding=0, bn=False),
            conv(out_channels, num_heatmaps, kernel_size=1, padding=0, bn=False, relu=False)
        )
        self.pafs = nn.Sequential(
            conv(out_channels, out_channels, kernel_size=1, padding=0, bn=False),
            conv(out_channels, num_pafs, kernel_size=1, padding=0, bn=False, relu=False)
        )

    def forward(self, x):
        trunk_features = self.trunk(x)
        heatmaps = self.heatmaps(trunk_features)
        pafs = self.pafs(trunk_features)
        return [heatmaps, pafs] #3.29 []


# Pose 3D
# ResNet NetWork
class ResBlock(nn.Module):
    def __init__(self, in_channels, out_channels, ratio, should_align=False):
        super(ResBlock, self).__init__()
        self.should_align = should_align
        self.bottleneck = nn.Sequential(
            conv(in_channels, in_channels // ratio, kernel_size=1, padding=0),
            conv(in_channels // ratio, in_channels // ratio),
            conv(in_channels // ratio, out_channels, kernel_size=1, padding=0)
        )
        if self.should_align:
            self.align = conv(in_channels, out_channels, kernel_size=1, padding=0)

    def forward(self, x):
        res = self.bottleneck(x)
        if self.should_align:
            x = self.align(x)
        return x + res

class RefinementStageLight(nn.Module):
    def __init__(self, in_channels, mid_channels, out_channels):
        super(RefinementStageLight, self).__init__()
        self.trunk = nn.Sequential(
            RefinementStageBlock(in_channels, mid_channels),
            RefinementStageBlock(mid_channels, mid_channels)
        )
        self.feature_maps = nn.Sequential(
            conv(mid_channels, mid_channels, kernel_size=1, padding=0, bn=False),
            conv(mid_channels, out_channels, kernel_size=1, padding=0, bn=False, relu=False)
        )

    def forward(self, x):
        trunk_features = self.trunk(x)
        feature_maps = self.feature_maps(trunk_features)
        return feature_maps


class Pose3D(nn.Module):
    def __init__(self, in_channels, num_2d_heatmaps, ratio=2, out_channels=57):
        super(Pose3D, self).__init__()
        self.stem = nn.Sequential(
            ResBlock(in_channels + num_2d_heatmaps, in_channels, ratio, should_align=True),
            ResBlock(in_channels, in_channels, ratio),
            ResBlock(in_channels, in_channels, ratio),
            ResBlock(in_channels, in_channels, ratio),
            ResBlock(in_channels, in_channels, ratio),
        )
        self.prediction = RefinementStageLight(in_channels, in_channels, out_channels)

    def forward(self, x, feature_maps_2d):
        # torch.cat concatnate two tensor
        # 0, 1 represent row, col respectively

        stem = self.stem(torch.cat([x, feature_maps_2d], 1))
        # channels error
        feature_maps = self.prediction(stem)
        return feature_maps


class PoseEstimationWithMobileNet(nn.Module):
    def __init__(self, num_refinement_stages=1, num_channels=128, num_heatmaps=19, num_pafs=38):
        super().__init__()
        self.model = nn.Sequential(
            conv(     3,  32, stride=2, bias=False),
            conv_dw( 32,  64),
            conv_dw( 64, 128, stride=2),
            conv_dw(128, 128),
            conv_dw(128, 256, stride=2),
            conv_dw(256, 256),
            conv_dw(256, 512),  # conv4_2
            conv_dw(512, 512, dilation=2, padding=2),
            conv_dw(512, 512),
            conv_dw(512, 512),
            conv_dw(512, 512),
            conv_dw(512, 512)   # conv5_5
        )
        self.cpm = Cpm(512, num_channels)

        self.initial_stage = InitialStage(num_channels, num_heatmaps, num_pafs)
        self.refinement_stages = nn.ModuleList()
        for idx in range(num_refinement_stages):
            self.refinement_stages.append(RefinementStage(num_channels + num_heatmaps + num_pafs, num_channels,
                                                          num_heatmaps, num_pafs))

        # 先训练Mobilnet， 关闭Pose3D network。
        self.Pose3D = Pose3D(128, num_2d_heatmaps=57)

    def forward(self, x):
        backbone_features = self.model(x)
        backbone_features = self.cpm(backbone_features)

        stages_output = [*self.initial_stage(backbone_features)]
        #stages_output = self.initial_stage(backbone_features) #3.29 
        for refinement_stage in self.refinement_stages:
            stages_output.extend(
                refinement_stage(torch.cat([backbone_features, stages_output[-2], stages_output[-1]], dim=1)))

        # return stages_output

        # Pose inference:
        #   1. First, infer 2D joint pixel locations P_2D and joint confidence S for each person by openpose.
        #   2. Second, use 2D location and joint confidence in conjunction with ORPMs M to infer the 3D pose of all person.
        
        keypoints2d_maps = stages_output[-2]
        paf_maps = stages_output[-1]
        
        # problem
        # use 2D joint heatmaps to predict 3D pose
        # use pafs represent a 2D vector field pointing from a joint of type j to its parent.
        # single-shot is that predict a fixed number of maps in a single forward pass irrespective of the number of persons in the scene.
        
        location_maps = self.Pose3D(backbone_features, torch.cat([stages_output[-2], stages_output[-1]], dim=1))
        return  location_maps, keypoints2d_maps, paf_maps