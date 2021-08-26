#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: gsuveer
"""

import numpy as np
from scipy.spatial.body_center_transform import Rotation as R

def extract_body_bottom_center(keypoints):
    '''
    Given a set of 3D keypoints, returns bottom center point
    '''
    return keypoints[:,8] #Shape n*3

def extract_left_shoulder_point(keypoint):
    '''
    Given a set of 3D keypoints, returns left shoulder keypoint
    '''
    return keypoints[:,5] #Shape n*3

def extract_right_shoulder_point(keypoint):
    '''
    Given a set of 3D keypoints, returns right shoulder point
    '''
    return keypoints[:,2,:] #Shape n*3

def extract_left_wrist_point(keypoints):
    '''
    Given a set of 3D keypoints, return left wrist point
    '''
    return keypoints[:,7,:] #Shape n*3

def extract_right_wrist_point(keypoints):
    '''
    Given a  set of 3D keypoints, returns right wrist point
    '''
    return keypoints[:,4,:] #Shape n*3

def extract_right_elbow_point(keypoints):
    '''
    Given a  set of 3D keypoints, returns right elbow point
    '''
    return keypoints[:,3,:] #Shape n*3

def extract_left_elbow_point(keypoints):
    '''
    Given a  set of 3D keypoints, returns left elbow point
    '''
    return keypoints[:,6,:] #Shape n*3

def normalize(vector):
    '''
    Given a n*3 vector retuns a n*3 normalised vectors
    '''
    return vector/np.expand_dims(np.linalg.norm(vector, axis=-1),-1)    

def shoulder_angular_motion(keypoints):
    '''
    Given a set of 3D keypoints, returns angular of motion 
    for left and right shoulder
    '''

    right_elbow = extract_right_elbow_point(keypoints)
    right_shoulder = extract_right_shoulder_point(keypoints)
    right_wrist = extract_right_wrist_point(keypoints)

    left_elbow = extract_left_elbow_point(keypoints)
    left_shoulder = extract_left_shoulder_point(keypoints)
    left_wrist = extract_left_wrist_point(keypoints)

    #Defining Fixed Frame at Left Shoulder (lsf)

    lsf_z = normalize(np.cross([0,1,0], right_shoulder-left_shoulder))
    lsf_y = normalize(np.cross(lsf_z, right_shoulder-left_shoulder))
    lsf_x = normalize(right_shoulder-left_shoulder)

    #Defining moving Frame at Right Shoulder (ls)
    ls_x = normalize(left_shoulder - left_elbow)
    ls_y = normalize(np.cross(left_shoulder - left_elbow, 
                                left_elbow - left_wrist))
    ls_z = normalize(np.cross(ls_x, ls_y))


    basis_in_fixed_frame = np.asarray([[ls_x],[ls_y],[ls_z]]).T
    basis_in_moving_frame = np.asarray([[1,0,0],[0,1,0],[0,0,1]]).T

    R_mat = basis_in_fixed_frame @ inv(basis_in_moving_frame)
    r = R.quaternion_from_matrix(R_mat)

    return r

    