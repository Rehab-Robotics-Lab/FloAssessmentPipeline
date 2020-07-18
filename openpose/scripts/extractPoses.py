#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul  8 23:07:00 2020

@author: gsuveer
"""

import sys
import cv2
import os
import argparse
import numpy as np

dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append('../python');

try:
    from openpose import pyopenpose as op
except ImportError as e:
    print('Error: OpenPose library could not be found. The path is probably not set correctly')
    raise e
    
params = dict()
params["model_folder"] = "../../models/"

'''
Function to take a Frame and return keypoints and output image with keypoints
'''
def processFrame(Image, opWrapper) :
    
    
    datum = op.Datum()
    datum.cvInputData = np.uint8(Image)
    opWrapper.emplaceAndPop([datum])
    cv2.imwrite('output/test.jpg', datum.cvOutputData)
    return datum.cvOutputData, datum.poseKeypoints

'''
Function to take in a array of RGB images and return a array of images 
with keypoints and a separate keypoint array

The last channel is taken as number of images
'''

def processFrames(Images):
    global params
    print(params)
    
    if len(Images.shape)<4 :
        Images = np.expand_dims(Images,-1)
    
    OutputImages = np.empty(Images.shape)
    num_images = 1
    
    try :
        num_images = Images.shape[3]
    except:
        pass
    
    OutputPoseKeypoints = np.zeros((num_images,25,3))
        
    opWrapper = op.WrapperPython()
    opWrapper.configure(params)
    opWrapper.start()

    for i in range(Images.shape[-1]):
        OutputImage, poseKeypoints = processFrame(Images[:,:,:,i], opWrapper)
        OutputImages[:,:,:,i] = OutputImage
        OutputPoseKeypoints[i,:,:] = poseKeypoints
    
    return OutputImages, OutputPoseKeypoints
