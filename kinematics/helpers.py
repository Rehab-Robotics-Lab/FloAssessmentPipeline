#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Aug 14 18:59:09 2020

@author: gsuveer
"""


import numpy as np

'''
Given a set of 3D keypoints, returns left shoulder points across time
'''
def left_shoulder_points(keypoints):
    return keypoints[:,5,:]

'''
Given a set of 3D keypoints, returns right shoulder points across time
'''
def right_shoulder_points(keypoints):
    return keypoints[:,2,:]

'''
Given a set of 3D keypoints, return left wrist points across time
'''
def left_wrist_points(keypoints):
    return keypoints[:,7,:]


'''
Given a  set of 3D keypoints, returns right wrist points across time
'''
def right_wrist_points(keypoints):
    return keypoints[:,4,:]



    