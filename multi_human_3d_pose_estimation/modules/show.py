import sys
sys.path.append('/home/xuchengjun/multi_human')
from tensorboardX import SummaryWriter
import torch
from models.with_mobilenet import PoseEstimationWithMobileNet

if __name__ == "__main__":
    net = PoseEstimationWithMobileNet()
    net.to('cuda:0')
    

    input_ = torch.rand(1,3,1080,1920)
    input_ = input_.to('cuda:0')
    output = net(input_)

    with SummaryWriter(comment='net') as w:
        w.add_graph(net,(input_,))