#!/usr/bin/env python
import numpy as np
import cv2
import rospy
import math
from cv_bridge import CvBridge


from sensor_msgs.msg import Image
from visualization_msgs.msg import MarkerArray
from point_match.msg import data_array


global image

def callback_img(img):
    global image
    image = img ### keep input img to global variable

def callback(boundingbox):
### append pointcloud boundingbox 3D location into data matrix 
    point_data = []
    len_input = len(boundingbox.markers)
    #print(boundingbox.bounding_boxes)
    for i in range(len_input):
        b = []
        b.append(boundingbox.markers[i].pose.position.x) ### lidar xyz
        b.append(boundingbox.markers[i].pose.position.y)
        b.append(boundingbox.markers[i].pose.position.z)
	point_data.append(b)  ### b = [x,y,z]

### declare projection matrix
### p is matrix for camera
### r is matrix for difference of coordinate axis
    p = [[693.5944, 0, 636.1532, 0], [0, 693.5944, 394.9452, 0], [0, 0, 1, 0]]

    x = 0
    y = 0
    z = 0
    c = 0  # roll 
    b = -3.201592 / 2  # pitch 
    a = 3.141592 / 2  # yaw 

    r = [[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]]
    r[0][0] = np.cos(a) * np.cos(b);
    r[0][1] = np.cos(a) * np.sin(b) * np.sin(c) - np.sin(a) * np.cos(c);
    r[0][2] = np.cos(a) * np.sin(b) * np.cos(c) + np.sin(a) * np.sin(c);

    r[1][0] = np.sin(a) * np.cos(b);
    r[1][1] = np.sin(a) * np.sin(b) * np.sin(c) + np.cos(a) * np.cos(c);
    r[1][2] = np.sin(a) * np.sin(b) * np.cos(c) - np.cos(a) * np.sin(c);

    r[2][0] = - np.sin(b);
    r[2][1] = np.cos(b) * np.sin(c);
    r[2][2] = np.cos(b) * np.cos(c);

    projection = np.array(p) ## camera
    rotation = np.array(r)   ## axis

### get input image message 
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(image) # Convert the message to a new image


### projection calculate

    array_for_save = data_array()

    for j in range(len_input):
        input_array = np.array([[point_data[j][0]], [point_data[j][1]], [point_data[j][2]], [1]]) # for calculate
        data2 = np.dot(projection, np.dot(rotation, input_array)) # projection * rotation * input (dot product)
        imgx = data2[0]/data2[2]
        imgy = data2[1]/data2[2]
        img = cv2.circle(img, (imgx, imgy), 10, (255,0,255), -1)

        print(j, " :", point_data[j])
        array_for_save.data.append(point_data[j][0])# z
        array_for_save.data.append(point_data[j][1])# y
        array_for_save.data.append(point_data[j][2])# z
        array_for_save.data.append(imgx[0])# imgx
        array_for_save.data.append(imgy[0])# imgy
        array_for_save.data.append(j+1) #ID
        array_for_save.data.append(1) #count
        array_for_save.data.append(0) #time
        array_for_save.data.append(0) # xx
        array_for_save.data.append(0) # xv
        array_for_save.data.append(0) # xp1
        array_for_save.data.append(0) # xp2
        array_for_save.data.append(0) # xp3
        array_for_save.data.append(0) # xp4
        array_for_save.data.append(0) # yx
        array_for_save.data.append(0) # yv
        array_for_save.data.append(0) # yp1
        array_for_save.data.append(0) # yp2
        array_for_save.data.append(0) # yp3
        array_for_save.data.append(0) # yp4





### send output back to ros message
    image_message = bridge.cv2_to_imgmsg(img, "8UC4") #8UC4 or bgr4
    pub.publish(image_message) ###########
    pub2.publish(array_for_save) ############

### 1d to 2d matrix
    length = len(array_for_save.data) / 20   #######################
    c = [[] for i in range(length)]
    k = -1
    for i in range(len(array_for_save.data)):
        if i%20 is 0:                        #######################
            k += 1
        c[k].append(array_for_save.data[i])
    print(c) ############



if __name__ == "__main__":  ### initialize ros node
    rospy.init_node('lidar_to_point')
    pub = rospy.Publisher('/point_match/lidar_point_image', Image, queue_size=20)
    pub2 = rospy.Publisher('/point_match/lidar_array', data_array, queue_size=20)
    sub = rospy.Subscriber('/point_process/boundingbox', MarkerArray, callback)
    sub = rospy.Subscriber('/zed/zed_node/rgb/image_rect_color', Image, callback_img)
    rospy.spin()
