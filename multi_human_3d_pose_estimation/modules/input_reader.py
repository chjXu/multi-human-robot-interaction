import os
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import message_filters
from IPython import embed

# it is use of a picture
class ImageReader(object):
    def __init__(self, file_names):
        self.file_name = file_names

    def __iter__(self):
        return self

    def __next__(self):
        img = cv2.imread(self.file_name)
        # img = cv2.resize(img, (960, 540))
        if img is None:
            raise IOError("Image Can not read".format(self.file_name))
        return img

class ImageSeqReader(object):
    def __init__(self, file_names):
        self.file_name = file_names
        self.max_idx = len(file_names)

    def __iter__(self):
        self.idx = 0
        return self

    def __next__(self):
        if self.idx == self.max_idx:
            raise StopIteration
        img = cv2.imread(self.file_names[self.idx], cv2.IMREAD_COLOR)
        if img.size == 0:
            raise IOError('Image {} cannot be read'.format(self.file_names[self.idx]))
        self.idx = self.idx + 1
        return img

class VideoReader(object):
    def __init__(self, file_name):
        self.file_name = file_name
        try:  # OpenCV needs int to read from webcam
            self.file_name = int(file_name)
        except ValueError:
            pass

    def __iter__(self):
        self.cap = cv2.VideoCapture(self.file_name)
        if not self.cap.isOpened():
            raise IOError('Video {} cannot be opened'.format(self.file_name))
        return self

    def __next__(self):
        was_read, img = self.cap.read()
        if not was_read:
            raise StopIteration
        #img = cv2.resize(img, (960, 540))
        return img

class CameraTopic(object):
    def __init__(self, topic_name):
        # rospy.init_node('Image_sub', anonymous=True)
        self.image = None
        self.cv_bridge = CvBridge()
        self.topic = topic_name
        self.image_sub = rospy.Subscriber(self.topic, Image, self.callback)

    def callback(self, msg):
        # rospy.loginfo('Image has received...')
        self.image = self.cv_bridge.imgmsg_to_cv2(msg)

    def __iter__(self):
        return self

    def __next__(self):
        if self.image is None:
            raise StopIteration
        # self.image = cv2.resize(self.image, (960, 540))
        return self.image



    # def __iter__(self):
    #     return self
    #
    # def __next__(self):
    #     if self.image_1 is None and self.image_2 is None:
    #         raise StopIteration
    #     # self.image = cv2.resize(self.image, (960, 540))
    #     return self.image_1, self.image_2
