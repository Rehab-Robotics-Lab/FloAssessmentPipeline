#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jul 12 13:03:20 2020

@author: gsuveer
"""


import numpy as np
import cv2 
from extractPoses import processFrame
from extractPoses import processFrames 

'''
Images = np.load('output/testdata.npy')

print(type(Images))
print(Images.shape)
print(Images.dtype)


for i in range(Images.shape[3]):
    cv2.imwrite('output/test'+str(i)+'.jpg', Images[:,:,:,i])
'''

img  = cv2.imread('test_data/test_img.jpg')
OutputImages, OutputPoseKeypoints = processFrames(img)

output = OutputImages[:,:,:,0]
kp = OutputPoseKeypoints[0]