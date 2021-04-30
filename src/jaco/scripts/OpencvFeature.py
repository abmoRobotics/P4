#!/usr/bin/env python3
import time
import pyfakewebcam
import roslib
import numpy as np
roslib.load_manifest('jaco')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
#from vision_opencv.cv_bridge.python import CvBridge, CvBridgeError
#from cv_bridge.boost.cv_bridge_boost import getCvType

class image_converter:

  def __init__(self):
    #self.image_pub = rospy.Publisher("image_topic_2",Image)

    
    self.image_sub = rospy.Subscriber("/Imagepub/RGB",Image,self.callback)

    self.camera = pyfakewebcam.FakeWebcam('/dev/video1', 640, 480)



  def imgmsg_to_cv2(self,img_msg):
        if img_msg.encoding != "bgr8":
            rospy.logerr("This Coral detect node has been hardcoded to the 'bgr8' encoding.  Come change the code if you're actually trying to implement a new camera")
        dtype = np.dtype("uint8") # Hardcode to 8 bits...
        dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
        image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                    dtype=dtype, buffer=img_msg.data)
    # If the byt order is different between the message and the system.
        if img_msg.is_bigendian == (sys.byteorder == 'little'):
            image_opencv = image_opencv.byteswap().newbyteorder()
        return image_opencv    


  def callback(self,data):
    
    cv_image = self.imgmsg_to_cv2(data)
    self.camera.schedule_frame(cv_image)
    time.sleep(1/30.0)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

  

       


def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

main(sys.argv)