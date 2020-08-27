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
<<<<<<< HEAD
#import openpose.scripts.extractPoses as kp
=======
>>>>>>> a46c2e22fa7ab7513598d80ea68387eca1b6f1d7
#Reading data from HDF5

hf = h5py.File('/media/gsuveer/391cd01c-d5a2-4313-947a-da8978447a80/gsuveer/Desktop/Flo_data/experiment1.hdf5', 'r')
print(hf.keys())

<<<<<<< HEAD
dset = hf['Experiment_1/Video/lower_realsense/color/group_1']
=======
dset = hf['Experiment_1/Video/lower_realsense/group_1']
>>>>>>> a46c2e22fa7ab7513598d80ea68387eca1b6f1d7
print(dset.shape)
print(type(dset))
images = np.asarray(dset)
print(images.shape)
print(type(images))
timestamps_secs = dset.attrs.get("timestamps_secs")
weights = "emotion_recognition/src/model.h5"
predicted_emotion, prediction_scores_array = emo.extract_emotions(images, weights)
height,width = images.shape[0], images.shape[1]
video=cv2.VideoWriter('/media/gsuveer/391cd01c-d5a2-4313-947a-da8978447a80/gsuveer/Desktop/Flo_data/video.avi',cv2.VideoWriter_fourcc(*'MJPG'),
                      20,(width,height))

print(len(predicted_emotion))
print(predicted_emotion[0])

#Run docker here
<<<<<<< HEAD
'''
=======

>>>>>>> a46c2e22fa7ab7513598d80ea68387eca1b6f1d7
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
<<<<<<< HEAD
video.release()
'''

=======
video.release()
>>>>>>> a46c2e22fa7ab7513598d80ea68387eca1b6f1d7
