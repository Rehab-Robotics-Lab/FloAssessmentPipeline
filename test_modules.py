#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Aug 23 18:17:57 2020
@author: gsuveer
"""

import numpy as np
import h5py
import cv2
import emotion_recognition.src.emotion_recognition as emo
import kinematics.extract_profiles as kp

hf = h5py.File('/media/gsuveer/391cd01c-d5a2-4313-947a-da8978447a80/gsuveer/Desktop/Flo_data/experiment1.hdf5', 'r')

dset = hf['Experiment_1/Video/lower_realsense/color/group_1']
images = np.asarray(dset)
timestamps_secs = dset.attrs.get("timestamps_secs")
K = dset.attrs.get("K")

dset = hf['Experiment_1/Video/lower_realsense/depth/group_1']
depth = np.asarray(dset)
depth_timestamps_secs = dset.attrs.get("timestamps_secs")


'''
weights = "emotion_recognition/src/model.h5"
predicted_emotion, prediction_scores_array = emo.extract_emotions(images, weights)
height,width = images.shape[0], images.shape[1]
'''
#Run docker here

'''
video=cv2.VideoWriter('/media/gsuveer/391cd01c-d5a2-4313-947a-da8978447a80/gsuveer/Desktop/Flo_data/video.avi',cv2.VideoWriter_fourcc(*'MJPG'),
                      20,(width,height))
images = np.load('/media/gsuveer/391cd01c-d5a2-4313-947a-da8978447a80/gsuveer/Desktop/Flo_data/testdata.npy')

for i in range(images.shape[3]):
    image = cv2.putText(np.float32(images[:,:,:,i]), 
                        predicted_emotion[i], 
                        (30,30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 
                        1, 
                        (255,0,0), 
                        2)
    #print(image.dtype)
    #print(image.shape)
    video.write(np.uint8(image))

cv2.destroyAllWindows()
video.release()
'''

keypoints = np.load('/media/gsuveer/391cd01c-d5a2-4313-947a-da8978447a80/gsuveer/Desktop/Flo_data/testdata_keypoints.npy')
K_inv = np.linalg.inv(K)
#Depth Keypoint synchronization
for i in range(keypoints.shape[0]):
    time = timestamps_secs[i]
    idx  = np.argmin(np.absolute(depth_timestamps_secs-time))
    depth_image_sync = depth[:,:,idx]
    #Possible interpolation error?
    #Slight error might also be introduced due to timestamp slicing
    z = depth_image_sync[np.uint8(keypoints[i,:,0]),np.uint8(keypoints[i,:,1])]
    keypoints[i,:,0] = np.multiply(keypoints[i,:,0] , z)
    keypoints[i,:,1] = np.multiply(keypoints[i,:,1] , z)
    keypoints[i,:,2] = z
    #25 * 3
    kp_ = keypoints[i,:,:]
    print(kp_.shape)
    print(K_inv.shape)
    #Points in camera frame
    kp_c= np.matmul(K_inv, kp_.T)  

print("x:", keypoints[0,:,0])
print("y:", keypoints[0,:,1])
print("z:", keypoints[0,:,2])
    
    

