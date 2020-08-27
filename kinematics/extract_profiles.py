#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Aug  8 16:52:54 2020

@author: gsuveer
"""

import numpy as np
from scipy.spatial import ConvexHull, convex_hull_plot_2d

'''
Given a set of 3D keypoints, returns left shoulder points across time
'''
def extract_left_shoulder_points(keypoints):
    return keypoints[:,5,:]

'''
Given a set of 3D keypoints, returns right shoulder points across time
'''
def extract_right_shoulder_points(keypoints):
    return keypoints[:,2,:]

'''
Given a set of 3D keypoints, return left wrist points across time
'''
def extract_left_wrist_points(keypoints):
    return keypoints[:,7,:]


'''
Given a  set of 3D keypoints, returns right wrist points across time
'''
def extract_right_wrist_points(keypoints):
    return keypoints[:,4,:]

'''
Fuction that takes in a 3D keypoints and timestamps
returns a velocity profile with timestamps 

keypoints shape (n * 25 * 3)
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
    
    max_xyz = np.amax(keypoints, axis=0)
    min_xyz = np.amin(keypoints, axis=0)
    
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

'''
Funtion to calculate path(Sum of Distances) from 3D keypoints
Returns Distance Moved
keypoints shape (n * 25 * 3)
'''

def path(keypoints):
    distance= np.linalg.norm(keypoints, axis=-1)  #should be (n*25)
    return np.sum(distance, axis=0) #shape 25

'''
Funtion to calculate range of movement from 3D keypoints and time_stamps
Returns the difference between max and minimum distance
keypoints shape (n * 25 * 3)
'''

def range_of_movement(keypoints):
    distance= np.linalg.norm(keypoints, axis=-1)  #should be (n*25)
    max_xyz = np.amax(keypoints, axis=0)
    min_xyz = np.amin(keypoints, axis=0)
    return max_xyz - min_xyz #shape 25

'''
Angular motion
'''

def angular_motion(keypoints):
    
    pass

'''
Reachable workspace 
Takes in 3D keypoints and calculates volume of Convex Hull traced by left and right wrist keypoints
Returns volume of each convex hull in metric scale
'''
def reachable_workspace(keypoints):
    right_wrist_points = extract_right_wrist_points(keypoints)
    left_wrist_points = extract_left_wrist_points(keypoints)
    
    right_wrist_hull = ConvexHull(right_wrist_points)
    left_wrist_hull  = ConvexHull(left_wrist_points)
    
    return left_wrist_hull.volume, right_wrist_hull.volume

