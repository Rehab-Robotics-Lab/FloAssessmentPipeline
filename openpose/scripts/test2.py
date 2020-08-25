#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jul 12 13:03:20 2020

@author: gsuveer
"""


import numpy as np
import cv2 
from extractPoses import processFrame

img  = cv2.imread('test_data/test_img.jpg')
OutputImages, OutputPoseKeypoints = processFrames(img)

output = OutputImages[:,:,:,0]
kp = OutputPoseKeypoints[0]
