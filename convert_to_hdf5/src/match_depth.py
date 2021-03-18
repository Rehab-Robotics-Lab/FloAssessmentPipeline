#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri March 23:21:43 2021

@author: gsuveer

Module to take a hdf5 database and

Attributes:
    directory_list: List of directories that contain hdf5 files

TODO

"""


import glob
import numpy as np
import h5py

def getfilename(directory):
    for f in directory:
        hdf5_files = glob.glob(f+"/*.hdf5")


def matchdepth(hdf5_object):
    """
    Function takes hdf5 object and saves closes depth image index for each color image
    Parameters
    ----------
    hdf5_object
    """


