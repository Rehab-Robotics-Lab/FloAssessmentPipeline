#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Aug  8 16:52:54 2020

@author: gsuveer
"""

import numpy as np
from scipy.spatial import ConvexHull, convex_hull_plot_2d

'''
Given a set of 3D keypoints, returns bottom center points across time
'''
def extract_body_bottom_center(keypoints):
    return keypoints[:,8,:] #Shape n*3

'''
Given a set of 3D keypoints, returns left shoulder points across time
'''
def extract_left_shoulder_points(keypoints):
    return keypoints[:,5,:] #Shape n*3

'''
Given a set of 3D keypoints, returns right shoulder points across time
'''
def extract_right_shoulder_points(keypoints):
    return keypoints[:,2,:] #Shape n*3

'''
Given a set of 3D keypoints, return left wrist points across time
'''
def extract_left_wrist_points(keypoints):
    return keypoints[:,7,:] #Shape n*3

'''
Given a  set of 3D keypoints, returns right wrist points across time
'''
def extract_right_wrist_points(keypoints):
    return keypoints[:,4,:] #Shape n*3

'''
Given a  set of 3D keypoints, returns right elbow points across time
'''
def extract_right_elbow_points(keypoints):
    return keypoints[:,3,:] #Shape n*3

'''
Given a  set of 3D keypoints, returns left elbow points across time
'''
def extract_left_elbow_points(keypoints):
    return keypoints[:,6,:] #Shape n*3


'''
Fuction that takes in a 3D keypoints and timestamps
returns a velocity profile with timestamps 

keypoints shape (n * 25 * 3)
'''
def velocity_profile(keypoints, timestamps):
    #Confirm Axis
    delta_xyz = np.diff(keypoints, axis=0)
    delta_t   = np.diff(timestamps)
    delta_t   = np.expand_dims(np.expand_dims(delta_t,-1),-1)
    velocity = np.divide(delta_xyz, delta_t)
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
    right_wrist_points = extract_right_wrist_points(keypoints)
    left_wrist_points = extract_left_wrist_points(keypoints)
    
    right_elbow_points = extract_right_elbow_points(keypoints)
    left_elbow_points = extract_left_elbow_points(keypoints)
    
    right_shoulder_points = extract_right_shoulder_points(keypoints)
    left_shoulder_points  = extract_left_shoulder_points(keypoints)
    
    right_elbow_basis_x = np.norm(right_wrist_points - right_elbow_points,axis=0)
    
    right_shoulder_basis_x = np.norm(right_elbow_points - right_shoulder_points, axis=0)
    
    right_shoulder_basis_y = np.cross(right_elbow_basis_x, right_shoulder_basis_x)
    right_elbow_basis_y    = right_shoulder_basis_y
    
    
    

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

'''
Body Center Transforms
'''
def body_center_transforms(keypoints):
    left_shoulder_points  = extract_left_shoulder_points(keypoints)
    right_shoulder_points = extract_right_shoulder_points(keypoints)
    bottom_body_points    = extract_body_bottom_center(keypoints)
    top_body_points       = (left_shoulder_points+right_shoulder_points)/2
    center_body_points    = (top_body_points + bottom_body_points)/2

    parallel_vector1 = left_shoulder_points - bottom_body_points
    parallel_vector1 = parallel_vector1/np.expand_dims(np.linalg.norm(parallel_vector1, axis=-1),-1) 
    
    parallel_vector2 = right_shoulder_points - bottom_body_points
    
    parallel_vector2 = parallel_vector2/np.expand_dims(np.linalg.norm(parallel_vector2, axis=-1),-1)
    '''
    print(parallel_vector1.shape)
    print(parallel_vector2.shape)
    print("Parallel vector 1", np.linalg.norm(parallel_vector2,axis =-1))
    print("Parallel vector 2", np.linalg.norm(parallel_vector1,axis =-1))
    '''
    basis_z = np.cross(parallel_vector1, parallel_vector2)
    basis_z = basis_z/np.expand_dims(np.linalg.norm(basis_z, axis=-1),-1)
    basis_y = top_body_points - bottom_body_points
    basis_y = basis_y/np.expand_dims(np.linalg.norm(basis_y,axis=-1),-1)
    
    #Not normalising basis x because it is the cross product of two orthogonal vectors
    basis_x = np.cross(basis_z, basis_y)
    
    
    print(basis_x.shape)
    print(basis_y.shape)
    print(basis_z.shape)
    print("basis_x", np.linalg.norm(basis_x,axis =-1))
    print("basis_y", np.linalg.norm(basis_y,axis =-1))
    print("basis_z", np.linalg.norm(basis_z,axis =-1))
    
    n = basis_x.shape[0]
    
    body_center_frame = np.zeros((3,3))
    camera_frame = np.asarray([[1,0,0],[0,1,0],[0,0,1]])
    camera_frame_inv = np.linalg.inv(camera_frame)
    
    T = np.zeros((4,4,n))
    
    for i in range(n):
        body_center_frame[:,0] = basis_x[i,:].T
        body_center_frame[:,1] = basis_y[i,:].T
        body_center_frame[:,2] = basis_z[i,:].T
        
        rotation = np.matmul(body_center_frame, camera_frame_inv)
        T[:3,:3,i]=rotation
        T[:3,3,i] = center_body_points[i,:]
    
    T[3,3,:] = 1
    return T
        
    
    
    