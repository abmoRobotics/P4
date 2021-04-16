
import cv2 as cv
import numpy as np
import argparse

Camera = cv.VideoCapture("http://127.0.0.1:5000/video_feed")

while True:
    frame, succes = Camera.read()
    cv.imshow("Camera Stream", frame)
    cv.waitKey(1)

