

import rospy
import cv2
import cv_bridge
import os
import sys

from sdk.Dataset import Dataset



if __name__ == '__main__' :
    testdata = Dataset('/home/sujiwo/Data/robotcar-dataset/2014-05-06-12-54-54')
    cam_ts = testdata.getTimestamp ('stereo')
    pass