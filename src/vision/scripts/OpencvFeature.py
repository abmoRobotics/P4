from __future__ import print_function
import cv2 as cv
import numpy as np
import argparse
#Import necessary libraries
from flask import Flask, render_template, Response
#Initialize the Flask app
app = Flask(__name__)
Camera = cv.VideoCapture(0)

sift = cv.SIFT_create()

params = cv.SimpleBlobDetector_Params()

# Change thresholds
params.minThreshold = 10
params.maxThreshold = 200

# Filter by Area.
params.filterByArea = True
params.minArea = 3000

# Filter by Circularity
params.filterByCircularity = True
params.minCircularity = 0.3

# Filter by Convexity
params.filterByConvexity = True
params.minConvexity = 0.5

# Filter by Inertia
params.filterByInertia = True
params.minInertiaRatio = 0.05

Feature = cv.SimpleBlobDetector_create(params)

def gen_frames():
    while True:
        #src = cv.imread('E:/Mine Ting/Dokumenter/P4/P4/src/vision/src/test.jpg')
        succes, src = Camera.read()
        #grey = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
    
        #kp = Feature.detect(src,None)
        #img = cv.drawKeypoints(src, kp, src)

        #for keypoint in kp:
        #    x = keypoint.pt[0]
        #    y = keypoint.pt[1]
        #    print("x: " + str(x) + "    y: " + str(y))
        #cv.imshow('img', img)
        ret, buffer = cv.imencode('.jpg', src)
        img = buffer.tobytes()
        yield (b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' + img + b'\r\n')  # concat frame one by one and show result
        cv.waitKey(2)


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame') 

if __name__ == "__main__":
    app.run(debug=True)   