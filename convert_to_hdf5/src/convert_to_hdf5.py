#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu May 28 23:21:43 2020

@author: gsuveer
"""

import os
import h5py
import numpy as np
import rospy
import rosbag
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

if __name__ == '__main__':

    rospy.init_node('rosbag_to_hdf5_node')
    rospy.loginfo('started rosbag to hdf5 conversion')    
    pre_processing_bag_file = rospy.get_param("~pre_processing_bag_file", None)
    rospy.loginfo('Bag file: %s', pre_processing_bag_file)        
    hdf5_filename = rospy.get_param("~hdf5_filename", None)
    rospy.loginfo('HDF5 Filename: %s', hdf5_filename)    
    mode = rospy.get_param("~mode", None)
    rospy.loginfo('Mode: %s', mode)    
    experiment_id = rospy.get_param("~experiment_metadata", None)
    rospy.loginfo('Experiment number: %s', experiment_id)    
    # Reading Bag File
    if not pre_processing_bag_file == "":
        try:
            pre_processing_bag = rosbag.Bag(pre_processing_bag_file)
        except:
            rospy.loginfo("BAG FILE COULD NOT BE READ")
    else:
        rospy.loginfo("No Argument provided for Bag Filename")        
    #Reading hdf5 File
    if not hdf5_filename == "":
        try:
            hdf5_database = h5py.File(hdf5_filename, mode)
        except:
            rospy.loginfo("HDF5 Database COULD NOT BE READ/CREATED")
    else:
        rospy.loginfo("No Argument provided for HDF5 Filename")
    experiment_subgroup = hdf5_database.create_group(experiment_id)
    video_subgroup = experiment_subgroup.create_group('Video')
    topics = ["/upper_realsense/color/image_raw",
              "/lower_realsense/color/image_raw",
             ]    
    topics_metainfo = ["/upper_realsense/color/camera_info", 
                       "/lower_realsense/color/camera_info", 
                       "/fisheye_cam/camera_info"]    
    bridge = CvBridge()
    meta_info = {}
    for topic in topics_metainfo:
        rospy.loginfo('Extracting info from : %s', topic)
        for tp, msg, t in pre_processing_bag.read_messages([topic]):
             meta_info[tp.split('/')[1]] = msg 
             break
    for topic in topics:
        rospy.loginfo('Extracting info from : %s', topic)        
        n = pre_processing_bag.get_message_count(topic)
        topic_meta_info = meta_info[topic.split('/')[1]]
        data = np.zeros((topic_meta_info.height, topic_meta_info.width, 3, n))
        i = 0         
        for tp, msg, t in pre_processing_bag.read_messages([topic]):             
             data[:,:,:,i] = bridge.imgmsg_to_cv2(msg, "rgb8")   
             i = i+1             
        rospy.loginfo("Number of Frames Extracted : %d", i)     
        dataset = video_subgroup.create_dataset(topic, data=data)
        dataset.attrs.create("height", topic_meta_info.height)
        dataset.attrs.create("width", topic_meta_info.width)
        dataset.attrs.create("D", topic_meta_info.D)
        dataset.attrs.create("K", topic_meta_info.K)
        dataset.attrs.create("R", topic_meta_info.R)
        dataset.attrs.create("P", topic_meta_info.P)
        dataset.attrs.create("binning_x", topic_meta_info.binning_x)
        dataset.attrs.create("binning_y", topic_meta_info.binning_y)
        
        rospy.loginfo("Exiting Process")

    