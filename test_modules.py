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
import tf
import rospy

hf = h5py.File('/media/gsuveer/391cd01c-d5a2-4313-947a-da8978447a80/gsuveer/Desktop/Flo_data/experiment1.hdf5', 'r')

dset = hf['Experiment_1/Video/lower_realsense/color/group_1']
images = np.asarray(dset)
print(images.shape)
timestamps_secs = dset.attrs.get("timestamps_secs")
K = dset.attrs.get("K")
#How to deal with time
dset = hf['Experiment_1/Video/lower_realsense/depth/group_1']
depth = np.asarray(dset)
depth_timestamps_secs = dset.attrs.get("timestamps_secs")
print(depth.shape)
K_inv = np.linalg.inv(np.reshape(K,(3,3)))
print("K_inv", K_inv)
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
print("K: " ,np.reshape(K,(3,3)))
K_inv = np.linalg.inv(np.reshape(K,(3,3)))
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
    kp_c= np.matmul(K_inv, kp_.T)  
    keypoints[i,:,:] = kp_c.T
    
#print("x:", keypoints[0,:,0])
#print("y:", keypoints[0,:,1])
#print("z:", keypoints[0,:,2])
    
#print("Range of Motion: ", kp.range_of_motion(keypoints))
#print("Distance Moved: ", kp.path(keypoints))
#print("Velocity Profile", kp.velocity_profile(keypoints, timestamps_secs))

T = kp.body_center_transforms(keypoints)


#TF broadcaster
rospy.init_node("TF_Broadcaster")
br = tf.TransformBroadcaster()
for i in range(T.shape[-1]):
    
    q = tf.transformations.quaternion_from_matrix(T[:,:,i])
    q = q/(q[0]**2+q[1]**2+q[2]**2+q[3]**2)
    print("Sending Transform: ", q[0]**2+q[1]**2+q[2]**2+q[3]**2)
    br.sendTransform((T[0,3,i],T[1,3,i],T[2,3,i]),
                     q,
                     rospy.Time(),"body","world")
    rospy.sleep(0.1)






