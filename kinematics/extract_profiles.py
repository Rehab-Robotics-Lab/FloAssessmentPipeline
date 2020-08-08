#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Aug  8 16:52:54 2020

@author: gsuveer
"""

import numpy as np

'''
Fuction that takes in a 3D keypoints and timestamps
returns a velocity profile with timestamps 
'''
def velocity_profile(keypoints, timestamps):
    #Confirm Axis
    delta_xyz = np.diff(keypoints, axis=0)
    delta_t   = np.diff(timestamps)
    
    velocity = delta_xyz/delta_t
    timestamps = timestamps - timestamps[0]
    
    return velocity, timestamps[:timestamps.shape[0]-1] 

'''
Fuction that takes in velocity_profile and timestamps
Return time to maximum velocity 
'''

def time_to_max_velocity(velocity, timestamps):
    norm_velocity = np.linalg.norm(velocity, axis=0)
    max_idx = np.argmax(norm_velocity)
    return timestamps[max_idx]

'''
Function to take in 3D keypoints and returns range in xyz
'''

def range_of_motion(keypoints):
    
    max_xyz = np.amax(keypoints, axis=1)
    min_xyz = np.amin(keypoints, axis=1)
    
    return max_xyz - min_xyz

'''
Function to take in velocity profile and timestamps
Returns jerk
'''

def jerk(velocity, timestamps):
    #Confirm Axis
    delta_xyz = np.diff(velocity, axis=0)
    delta_t   = np.diff(timestamps)
    acc = delta_xyz/delta_t
    timestamps = timestamps - timestamps[0]
    timestamps = timestamps[:timestamps.shape[0]-1]
    
    delta2_xyz = np.diff(delta_xyz, axis=0)
    delta2_t   = np.diff(timestamps)
    
    jerk = delta2_xyz/delta2_t
    
    return jerk
     