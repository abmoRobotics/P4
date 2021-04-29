from __future__ import print_function
import cv2 as cv
import numpy as np
import argparse
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
#Import necessary libraries
from flask import Flask, render_template, Response
#Initialize the Flask app
app = Flask(__name__)

bridge = CvBridge()

def LoadRos():
    app.run(debug=True)
    try:
      cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
def gen_frames():
    while True:
        #src = cv.imread('E:/Mine Ting/Dokumenter/P4/P4/src/vision/src/test.jpg')
        #succes, src = Camera.read()
        #grey = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
    
        #kp = Feature.detect(src,None)
        #img = cv.drawKeypoints(src, kp, src)

        #for keypoint in kp:
        #    x = keypoint.pt[0]
        #    y = keypoint.pt[1]
        #    print("x: " + str(x) + "    y: " + str(y))
        #cv.imshow('img', img)
        ret, buffer = cv.imencode('.jpg', cv_image)
        img = buffer.tobytes()
        yield (b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' + img + b'\r\n')  # concat frame one by one and show result
        cv.waitKey(2)

image_sub = rospy.Subscriber("/Imagepub/RGB",Image,LoadRos)
@app.route('/')
def index():
    return render_template('index.html')


@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame') 

if __name__ == "__main__":
    rospy.init_node('image_converter', anonymous=False)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")   