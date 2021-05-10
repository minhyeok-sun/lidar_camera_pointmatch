#!/usr/bin/env python
import rospy
import math
import numpy as np
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from visualization_msgs.msg import MarkerArray
from point_match.msg import data_array
from darknet_ros_msgs.msg import BoundingBoxes

global image

def callback_img(img):
    global image
    image = img ### keep input img to global variable

def callback(boundingbox):
### append pointcloud boundingbox 3D location into data matrix 
    point_data = [] # for calculate
    data_to_send = [] # for send
    len_input = len(boundingbox.bounding_boxes)

    for i in range(len_input):
        b = []
        b.append((boundingbox.bounding_boxes[i].xmin + boundingbox.bounding_boxes[i].xmax)/2)
        b.append((boundingbox.bounding_boxes[i].ymin + boundingbox.bounding_boxes[i].ymax)/2)
        b.append(boundingbox.bounding_boxes[i].id)
        b.append(boundingbox.bounding_boxes[i].probability)
	point_data.append(b)

        data_to_send.append((boundingbox.bounding_boxes[i].xmin + boundingbox.bounding_boxes[i].xmax)/2)
        data_to_send.append((boundingbox.bounding_boxes[i].ymin + boundingbox.bounding_boxes[i].ymax)/2)
        data_to_send.append(boundingbox.bounding_boxes[i].id)
        data_to_send.append(boundingbox.bounding_boxes[i].probability)

    print data_to_send


### get input image message 
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(image)
#    img = np.zeros((1080, 1920, 3), np.uint8)

#    final_array = data_array()

    for j in range(len_input):
        input_array = np.array([point_data[j][0], point_data[j][1]])
        imgx = input_array[0]
        imgy = input_array[1]
        if point_data[j][2] == 2:
            color = np.array([0,255,0])
        else:
            color = np.array([255,0,0])
        img = cv2.circle(img, (imgx, imgy), 10, color, -1)

### send output back to ros message
    image_message = bridge.cv2_to_imgmsg(img, "8UC4") #8UC4 or bgr8
    pub.publish(image_message)
    pub2.publish(data_to_send)

'''
    length = len(array_for_save.data) / 4
    c = [[] for i in range(length)]
    k = -1
    for i in range(len(array_for_save.data)):
        if i%4 is 0:
            k += 1
        c[k].append(array_for_save.data[i])
    print(c)
'''


if __name__ == "__main__":  ### initialize ros node
    rospy.init_node('yolo_to_point')
    pub = rospy.Publisher('/point_match/yolo_point_image', Image, queue_size=20)
    pub2 = rospy.Publisher('/point_match/yolo_array', data_array, queue_size=20)
    sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, callback)
    sub = rospy.Subscriber('/zed/zed_node/rgb/image_rect_color', Image, callback_img)
    rospy.spin()
