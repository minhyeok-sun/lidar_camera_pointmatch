#!/usr/bin/env python
import numpy as np
import cv2
import rospy
import math

from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from point_match.msg import data_array
from darknet_ros_msgs.msg import BoundingBoxes



def callback(matched_list):

    matched_num = len(matched_list.data)/6
    result_0 = np.reshape(matched_list.data, (matched_num,6)) # reshape (Car+None)
    print result_0
    print result_0.shape
    
    marker_array = MarkerArray()    
    
    for i in range(matched_num):
        marker = Marker()   

        marker.header.frame_id = "/velodyne"
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
   	marker.ns = 'a';
 	marker.id = i;
        marker.scale.x = 3.0
        marker.scale.y = 2.0
        marker.scale.z = 2.0
        marker.color.a = 1.0

        marker.pose.orientation.w = 1.0
        marker.pose.position.x = result_0[i][0]
        marker.pose.position.y = result_0[i][1] 
        marker.pose.position.z = result_0[i][2]
        marker.lifetime = rospy.Duration(0.1)
        marker_array.markers.append(marker)

        if result_0[i][5] == 2:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        else:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
    
    pub.publish(marker_array)

def callback_2(matched_list):

    matched_num = len(matched_list.data)/20
    result_0 = np.reshape(matched_list.data, (matched_num,20)) # reshape (Car+None)
    print result_0
    print result_0.shape
    
    marker_array = MarkerArray()    
    
    for i in range(matched_num):
        marker = Marker()   

        marker.header.frame_id = "/velodyne"
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
   	marker.ns = 'a';
 	marker.id = i;
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = result_0[i][0]
        marker.pose.position.y = result_0[i][1] 
        marker.pose.position.z = result_0[i][2]
        marker.lifetime = rospy.Duration(0.1)
        marker_array.markers.append(marker)
    
    pub2.publish(marker_array)

if __name__ == "__main__":  ### initialize ros node

    rospy.init_node('matched_boxing')

    pub = rospy.Publisher('/point_match/matched_box', MarkerArray, queue_size=20)
    sub = rospy.Subscriber('/point_match/matched_array', data_array, callback)
    pub2 = rospy.Publisher('/point_match/matched_box_ori', MarkerArray, queue_size=20)
    sub2 = rospy.Subscriber('/point_match/matched_array_ori', data_array, callback_2)

    rospy.spin()

