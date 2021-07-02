from torch import nn

# Convolutional
def conv(in_channels, out_channels, kernel_size=3, padding=1, bn=True, dilation=1, stride=1, relu=True, bias=True):
    # Conv2d will deal the 2 dim data
    # in_channels: image_channels
    # out_channels: out channels that adjust by model
    # kernel_size: may be int or tuple type, 2 means (2, 2), (2, 3) means (2, 3)
    # stride: step length
    # padding: zero padding
    # h / w = (h / w - kernel_size + 2 * padding)/stride + 1
    modules = [nn.Conv2d(in_channels, out_channels, kernel_size, stride, padding, dilation, bias=bias)]
    if bn:
        # BatchNorm2d: according to data's mean and var to normalize data
        # x_new = (1 - momentum) * x(history data) + momentum * x_t(new batch data)
        # momentum means the wight of new batch data
        modules.append(nn.BatchNorm2d(out_channels))
    if relu:
        # relu function add non-linear data
        # inplace=True means can change the data value.
        # Relu(x) = { x  if x > 0
        #           { 0  if x <= 0
        modules.append(nn.ReLU(inplace=True))
    return nn.Sequential(*modules)

# Convolutional and relu
def conv_dw(in_channels, out_channels, kernel_size=3, padding=1, stride=1, dilation=1):
    return nn.Sequential(
        nn.Conv2d(in_channels, in_channels, kernel_size, stride, padding, dilation=dilation, groups=in_channels, bias=False),
        nn.BatchNorm2d(in_channels),
        nn.ReLU(inplace=True),

        nn.Conv2d(in_channels, out_channels, 1, 1, 0, bias=False),
        nn.BatchNorm2d(out_channels),
        nn.ReLU(inplace=True),
    )

# Convolutional and elu and no batchnorm
def conv_dw_no_bn(in_channels, out_channels, kernel_size=3, padding=1, stride=1, dilation=1):
    return nn.Sequential(
        nn.Conv2d(in_channels, in_channels, kernel_size, stride, padding, dilation=dilation, groups=in_channels, bias=False),
        # The exponential linear unit with 0 < a is
        # f(x) = {  x               if x > 0            f'(x) = { 1         if x > 0
        #        {  a(exp(x)-1)     if x <=0                    { f(x) + a  if x <= 0
        nn.ELU(inplace=True),

        nn.Conv2d(in_channels, out_channels, 1, 1, 0, bias=False),
        nn.ELU(inplace=True),
    )
