import torch
import torch.nn as nn

class RefineNet_base(nn.Module):
    def __init__(self, in_dim=75, out_dim=45, flatten_size=1):
        super(RefineNet_base, self).__init__()
        self.layer_1 = nn.Sequential(
            nn.Linear(in_dim, 160*flatten_size),
            nn.BatchNorm1d(160*flatten_size),
            nn.ReLU()
        )
        self.layer_2 = nn.Sequential(
            nn.Linear(160 * flatten_size, 256 * flatten_size),
            nn.BatchNorm1d(256 * flatten_size),
            nn.ReLU()
        )
        self.layer_3 = nn.Sequential(
            nn.Linear(256 * flatten_size, 256 * flatten_size),
            nn.BatchNorm1d(256 * flatten_size),
            nn.ReLU()
        )
        self.layer_4 = nn.Sequential(
            nn.Linear(256 * flatten_size, 128 * flatten_size),
            nn.BatchNorm1d(128 * flatten_size),
            nn.ReLU()
        )
        self.layer_5 = nn.Linear(128 * flatten_size, out_dim)
        self.out_dim = out_dim

    def forward(self, input_x):
        x = self.layer_1(input_x)
        x = self.layer_2(x)
        x = self.layer_3(x)
        x = self.layer_4(x)
        output = self.layer_5(x)

        return output

class RefineNet(nn.Module):
    def __init__(self):
        super(RefineNet, self).__init__()
        self.block = RefineNet_base()

    def forward(self, input_x):
        output = self.block(input_x)
        return output