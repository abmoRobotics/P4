from __future__ import print_function
import cv2 as cv
import numpy as np
import argparse

Camera = cv.VideoCapture(0)
Feature = cv.xfeatures2d.LUCID_create()

while True:
    succes, img = Camera.read()
    grey = cv.cvtColor(img, cv.COLOR_RGB2GRAY)
    ift = cv.SIFT()
    kp = sift.detect(grey,None) 

    img=cv.drawKeypoints(grey,kp)

    cv2.imshow('sift_keypoints.jpg', img)

    cv.waitKey(1)
    