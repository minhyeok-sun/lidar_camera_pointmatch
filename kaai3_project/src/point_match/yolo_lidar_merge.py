#!/usr/bin/env python
import numpy as np
import cv2
import rospy
import math
import time
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker

from visualization_msgs.msg import MarkerArray
from point_match.msg import data_array
from darknet_ros_msgs.msg import BoundingBoxes

global lidar_data
global image

def sort(array, index, inverse):
    if inverse == 0:
        array = array[array[:, index].argsort()[::1]]
    else:
        array = array[array[:, index].argsort()[::-1]]
    return array


def callback_img(img):
    global image
    image = img

def callback_0(lidar_array):
    global lidar_data
    lidar_data = lidar_array.data



def callback_1(yolo_array):
    global lidar_data
    print('\033[96m')
    print('################################################################################################################################')
    print('\033[0m')


    yolo_num = len(yolo_array.data)/4 
    lidar_num = len(lidar_data)/20 

    yolo_xy = np.reshape(yolo_array.data, (yolo_num,4))
    lidar_xyz = np.reshape(lidar_data, (lidar_num,20))
    for i in range(lidar_xyz.shape[0]):
        lidar_xyz[i][5] = 0



    lidar_xyz = sort(lidar_xyz,0,0)
    yolo_xy = sort(yolo_xy,3,1)


    print('\033[92m')
    print(lidar_xyz)
    print('\033[0m')

    result_0 = [] # before real result
    result_A = [] # [X Y x y z ID]
    result_final = []

    matched_num = 0


    for i in range(yolo_num):
        D = []
        matched = []
        matched_c = []
        
        for j in range(lidar_num): # num of lidar car
            c = math.sqrt((yolo_xy[i][0] - lidar_xyz[j][3])**2 + (yolo_xy[i][1] - lidar_xyz[j][4])**2)

            if lidar_xyz[j][5] != 1 and lidar_xyz[j][0] < 5 and c < 150:
                matched.append(lidar_xyz[j][0])
                matched.append(lidar_xyz[j][1])
                matched.append(lidar_xyz[j][2])
                matched.append(yolo_xy[i][0])
                matched.append(yolo_xy[i][1])
                matched.append(yolo_xy[i][2])
                matched_c.append(c)


            elif lidar_xyz[j][5] != 1 and lidar_xyz[j][0] >=5 and lidar_xyz[j][0] < 10 and c < 100:
                matched.append(lidar_xyz[j][0])
                matched.append(lidar_xyz[j][1])
                matched.append(lidar_xyz[j][2])
                matched.append(yolo_xy[i][0])
                matched.append(yolo_xy[i][1])
                matched.append(yolo_xy[i][2])
                matched_c.append(c)


            elif lidar_xyz[j][5] != 1 and lidar_xyz[j][0] >= 10 and c < 50:
                matched.append(lidar_xyz[j][0])
                matched.append(lidar_xyz[j][1])
                matched.append(lidar_xyz[j][2])
                matched.append(yolo_xy[i][0])
                matched.append(yolo_xy[i][1])
                matched.append(yolo_xy[i][2])
                matched_c.append(c)


        print('\033[94m')
        print(i, matched)

        if not len(matched)==0:
            min_index = matched_c.index(min(matched_c))
            for i in range(6):
                result_0.append(matched[min_index*6+i])
            matched_num += 1


    print('')
    print('result_0', matched_num)

    print(result_0)
    print('\033[0m')
           

    result_A = np.reshape(result_0, (len(result_0)/6, 6)) # reshape (Car+None)
    result_final.append(result_A)

    result_final = np.asarray(result_final)



### get input image message 
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(image)

    for j in range(matched_num):
        input_array = [result_A[j][3], result_A[j][4]]
        imgx = int(input_array[0])
        imgy = int(input_array[1])
        color = np.array([0,0,255])
        img = cv2.circle(img, (imgx, imgy), 10, color, -1)



### send output back to ros message
    image_message = bridge.cv2_to_imgmsg(img, "8UC4") #8UC4 or bgr8

    pub.publish(result_final.reshape(-1))
    pub2.publish(image_message)


    a = np.asarray(lidar_data)
    pub3.publish(a)

if __name__ == "__main__":  ### initialize ros node

    rospy.init_node('yolo_lidar_merge')

    pub = rospy.Publisher('/point_match/matched_array', data_array, queue_size=20)
    pub3 = rospy.Publisher('/point_match/matched_array_ori', data_array, queue_size=20)
    pub2 = rospy.Publisher('/point_match/final_matched_image', Image, queue_size=20)


    sub = rospy.Subscriber('/point_match/yolo_array', data_array, callback_1)
    sub1 = rospy.Subscriber('/point_match/lidar_ID', data_array, callback_0)
    sub2 = rospy.Subscriber('/zed/zed_node/rgb/image_rect_color', Image, callback_img)
    rospy.spin()
