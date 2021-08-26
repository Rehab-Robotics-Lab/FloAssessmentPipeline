#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: gsuveer
"""

import numpy as np
from scipy.spatial import Rotation as R

def extract_body_bottom_center(keypoints):
    '''
    Given a set of 3D keypoints, returns bottom center point
    '''
    return keypoints[8,:] #Shape (,3)

def extract_left_shoulder_point(keypoints):
    '''
    Given a set of 3D keypoints, returns left shoulder keypoint
    '''
    return keypoints[5,:]

def extract_right_shoulder_point(keypoints):
    '''
    Given a set of 3D keypoints, returns right shoulder point
    '''
    return keypoints[2,:] #Shape n*3

def extract_left_wrist_point(keypoints):
    '''
    Given a set of 3D keypoints, return left wrist point
    '''
    return keypoints[7,:] #Shape n*3

def extract_right_wrist_point(keypoints):
    '''
    Given a  set of 3D keypoints, returns right wrist point
    '''
    return keypoints[4,:] #Shape n*3

def extract_right_elbow_point(keypoints):
    '''
    Given a  set of 3D keypoints, returns right elbow point
    '''
    return keypoints[3,:] #Shape n*3

def extract_left_elbow_point(keypoints):
    '''
    Given a  set of 3D keypoints, returns left elbow point
    '''
    return keypoints[6,:] #Shape n*3

def normalize(vector):
    '''
    Given a n*3 vector retuns a n*3 normalised vectors
    '''
    return vector/np.linalg.norm(vector)    

def right_shoulder_fixed_frame(keypoints):
    '''
    Returns fixed Frame at ls as a 3*3 matrix
    '''
    right_shoulder = extract_right_shoulder_point(keypoints)
    left_shoulder = extract_left_shoulder_point(keypoints)

    #Defining Fixed Frame at Left Shoulder (lsf)

    rsf_z = normalize(np.cross([0,1,0], left_shoulder - right_shoulder))
    rsf_y = normalize(np.cross(rsf_z, left_shoulder - right_shoulder))
    rsf_x = normalize(left_shoulder - right_shoulder)
    rs_fixed_frame = np.asarray([rsf_x, rsf_y, rsf_z]).T

    return rs_fixed_frame

def right_shoulder_moving_frame(keypoints):
    '''
    Returns moving Frame at ls as a 3*3 matrix
    '''
    left_elbow = extract_left_elbow_point(keypoints)
    left_shoulder = extract_left_shoulder_point(keypoints)
    left_wrist = extract_left_wrist_point(keypoints)
    
    #Defining moving Frame at Left Shoulder (ls)
    rs_x = normalize(left_shoulder - left_elbow)
    rs_y = normalize(np.cross(left_shoulder - left_elbow, 
                                left_elbow - left_wrist))
    rs_z = normalize(np.cross(ls_x, ls_y))
    rs_moving_frame = np.asarray([[ls_x],[ls_y],[ls_z]]).T

    return rs_moving_frame


def shoulder_angular_motion(keypoints):
    '''
    Given a set of 3D keypoints, returns angular of motion 
    for left and right shoulder
    '''
    fixed_frame = left_shoulder_fixed_frame(keypoints)
    moving_frame = left_shoulder_moving_frame(keypoints)

    
    R_mat = fixed_frame @ np.linalg.inv(moving_frame)
    r = R.quaternion_from_matrix(R_mat)

    return r

    