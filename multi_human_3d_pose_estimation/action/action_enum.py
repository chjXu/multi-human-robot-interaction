from enum import Enum

class Actions(Enum):
    T_pose = 0  #识别操作人员
    stand = 1   #
    sit = 2
    handing = 3
    operate = 4