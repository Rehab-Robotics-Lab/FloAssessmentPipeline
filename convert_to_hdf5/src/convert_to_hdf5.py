#!/usr/bin/env python3
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
        
    current_prefix = experiment_id+'/Video'
    
    
    image_topics = ["/upper_realsense/color/image_raw",
                    "/lower_realsense/color/image_raw",
                    "/upper_realsense/depth/image_rect_raw",
                    "/lower_realsense/depth/image_rect_raw",
                    "/fisheye_cam/image_raw"
              ]
    
    image_timestamps_secs_group = ["/upper_realsense/color/timestamp_secs",
                             "/lower_realsense/color/timestamp_secs",
                             "/upper_realsense/depth/timestamp_secs",
                             "/lower_realsense/depth/timestamp_secs",
                             "/fisheye_cam/timestamp_secs"
                             ]
    
    image_timestamps_nsecs_group = ["/upper_realsense/color/timestamp_nsecs",
                              "/lower_realsense/color/timestamp_nsecs",
                              "/upper_realsense/depth/timestamp_nsecs",
                              "/lower_realsense/depth/timestamp_nsecs",
                              "/fisheye_cam/timestamp_nsecs"
                              ]
    
    image_topics_metainfo = ["/upper_realsense/color/camera_info", 
                       "/lower_realsense/color/camera_info",
                       "/upper_realsense/depth/camera_info",
                       "/lower_realsense/depth/camera_info",
                       "/fisheye_cam/camera_info"]    
    
    bridge = CvBridge()
    initial_size = 0 
    if(mode == "w"):
        
        meta_info = {}
        for i, topic in enumerate(image_topics_metainfo):
            rospy.loginfo('Extracting meta info from : %s', topic)
            for tp, msg, t in pre_processing_bag.read_messages([topic]):
                 meta_info[image_topics[i]] = msg 
                 break
             
        datasets={}     
        for i, topic in enumerate(image_topics):
            rospy.loginfo('Creating new Datasets : %s' %(topic))
            topic_meta_info = meta_info[topic]
            n = pre_processing_bag.get_message_count(topic)
            if topic.split('/')[2] == "depth":
                 dataset = hdf5_database.create_dataset(current_prefix + topic, (initial_size, topic_meta_info.height,topic_meta_info.width), 
                                                        maxshape=(None,topic_meta_info.height,topic_meta_info.width),
                                                        dtype= np.uint16, chunks= True)
            else:
                dataset = hdf5_database.create_dataset(current_prefix + topic, (initial_size, topic_meta_info.height,topic_meta_info.width,3), 
                                                       maxshape=(None,topic_meta_info.height,topic_meta_info.width,3),
                                                       dtype=np.uint8, chunks= True)
                
            timestamp_secs = hdf5_database.create_dataset(current_prefix + image_timestamps_secs_group[i], (initial_size,), 
                                                           maxshape=(None,),
                                                           dtype= np.float32, chunks= True)
            
            timestamp_nsecs = hdf5_database.create_dataset(current_prefix + image_timestamps_nsecs_group[i], (initial_size,), 
                                                           maxshape=(None,),
                                                           dtype= np.float32, chunks= True)
            
            dataset.attrs.create("height", topic_meta_info.height)
            dataset.attrs.create("width", topic_meta_info.width)
            dataset.attrs.create("D", topic_meta_info.D)
            dataset.attrs.create("K", topic_meta_info.K)
            dataset.attrs.create("R", topic_meta_info.R)
            dataset.attrs.create("P", topic_meta_info.P)
            dataset.attrs.create("binning_x", topic_meta_info.binning_x)
            dataset.attrs.create("binning_y", topic_meta_info.binning_y)
            datasets[topic]=dataset
         
    for i, topic in enumerate(image_topics):
        rospy.loginfo('Extracting info from : %s' %(topic))        
        dataset = hdf5_database[current_prefix + topic]
        timestamp_secs = hdf5_database[current_prefix + image_timestamps_secs_group[i]]
        timestamp_nsecs = hdf5_database[current_prefix + image_timestamps_nsecs_group[i]]
        
        for tp, msg, t in pre_processing_bag.read_messages([topic]):        
            dataset.resize(dataset.shape[0]+1, axis=0)
            timestamp_secs.resize(timestamp_secs.shape[0]+1, axis=0)
            timestamp_nsecs.resize(timestamp_nsecs.shape[0]+1, axis=0)
            
            if topic.split('/')[2] == "depth":
                dataset[dataset.shape[0]-1,:,:] = bridge.imgmsg_to_cv2(msg,"16UC1")    
            else:
                dataset[dataset.shape[0]-1,:,:,:] = bridge.imgmsg_to_cv2(msg,"bgr8")
            
            timestamp_secs[timestamp_secs.shape[0]-1]   = msg.header.stamp.secs
            timestamp_nsecs[timestamp_nsecs.shape[0]-1] = msg.header.stamp.nsecs
                
    hdf5_database.close()            
    rospy.loginfo("Exiting Process")

    