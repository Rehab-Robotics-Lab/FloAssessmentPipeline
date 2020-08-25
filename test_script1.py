#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Aug 23 18:17:57 2020

@author: gsuveer
"""

import numpy as np
import h5py
import cv2
#Reading data from HDF5

hf = h5py.File('/media/gsuveer/391cd01c-d5a2-4313-947a-da8978447a80/gsuveer/Desktop/Flo_data/experiment1.hdf5', 'r')
print(hf.keys())

dset = hf['Experiment_1/Video/lower_realsense/group_1']
print(dset.shape)
print(type(dset))
images = np.asarray(dset)
print(images.shape)
print(type(images))
timestamps_secs = dset.attrs.get("timestamps_secs")
cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
weights = "model.h5"
extract_emotions(images, weights, cascade)
