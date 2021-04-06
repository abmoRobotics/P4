#!/usr/bin/env python
import argparse
import time
import cv_bridge
#from pathlib import Path

import rospy
import re
import numpy as np
import cv2
#import torch
#import torch.backends.cudnn as cudnn
from numpy import random
from vision.msg import Detection
from vision.msg import Detection_array


from sensor_msgs.msg import Image
#from models.experimental import attempt_load
#from utils.datasets import LoadStreams, LoadImages
#from utils.general import check_img_size, check_requirements, check_imshow, non_max_suppression, apply_classifier, scale_coords, xyxy2xywh, strip_optimizer, set_logging, increment_path
#from utils.plots import plot_one_box
#from utils.torch_utils import select_device, load_classifier, time_synchronized
from std_msgs.msg import Int16



if __name__ == '__main__':
    rospy.init_node('talker', anonymous=True)
    bridge = cv_bridge.CvBridge()
    video = "video4"
    pub = rospy.Publisher('stream',Image, queue_size=10)
    cap = cv2.VideoCapture(4)
    ret, frame = cap.read()

    
    # while(True):
    #     # Capture frame-by-frame
    #     ret, frame = cap.read()

    #     # Our operations on the frame come here
    #     #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    #     # Display the resulting frame
    #     cv2.imshow('frame',frame)
    #     if cv2.waitKey(1) & 0xFF == ord('q'):
    #         break

    # cap.release()
    # cv2.destroyAllWindows()    
    #print(frame)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    try:
        ros_image = bridge.cv2_to_imgmsg(frame,encoding="bgr8")
        original_image = bridge.imgmsg_to_cv2(ros_image,ros_image.encoding)
        cv2.imshow("frame", original_image)
        cv2.waitKey(10)
        #pub.publish(bridge.cv2_to_imgmsg(frame,"passthrough"))
        print("success")
    except cv_bridge.CvBridgeError as e:
        print(e)


