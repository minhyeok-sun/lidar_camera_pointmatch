#!/usr/bin/env python
import numpy as np
import cv2
import rospy
import math
import random
import threading
import time
import random
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from visualization_msgs.msg import MarkerArray
from point_match.msg import data_array
from darknet_ros_msgs.msg import BoundingBoxes
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

global image
global lidar_pub
lidar_pub = []   ### U time.time()pdate paper


def check(array):
    print("array")

    new_list = np.array([])
    for i in array:
        if i not in new_list:
            np.append(new_list,i,axis=0)

    array = array[array[:, 5].argsort()[::1]]
    return(array)

#########################################################################
###	Kalman filter function


def predict(x,v,P,dt):
    # x = F x
    # P = F P Ft + G Gt 
    X_predict = np.array([x, v])
    F = np.array([[1, dt],[0,1]])
    G = np.array([0.5*dt**2, dt]).reshape((2,1))      
    new_X = F.dot(X_predict) #(x,v)
    new_P = F.dot(P).dot(F.T) + G.dot(G.T)*0.1 #accel_variance = 0.1 fi
    return new_X[0],new_X[1],new_P

def update(x,v,P,Z):#x,v,p = existed variance / z,R measurment
    # y = z - H x
    # S = H P Ht + R
    # K = P Ht S^-1
    # x = x + K y
    # P = (I - K H) * P
    X_update = np.array([x,v])
    P = np.array(P)

    z = np.array([Z]) #input
    R = np.array([0.2**2]) #input
    H = np.array([1, 0]).reshape((1,2)) #No update
    y = z - H.dot(X_update) #No update
    S = H.dot(P).dot(H.T) + R #No update
    K = P.dot(H.T).dot(np.linalg.inv(S))
    new_X = X_update + K.dot(y) # UP (x,v)
    new_P = (np.eye(2) - K.dot(H)).dot(P) # UP(P)    

    return new_X[0],new_X[1],new_P



#########################################################################
###	initialize array or data for processing


def callback_0(lidar_array):
    global lidar_pub
    global kalman
    lidar_data = lidar_array.data # New lidar xyz []
    lidar_num = len(lidar_data)/20 # number of car in lidar
    lidar_xyz = np.reshape(lidar_data, (lidar_num,20)) # reshape
    now = time.time()  # time ****** should be modified - [1.61528466e+09]
    print('')
    print('0000000000000000000000000000000000000000000000000000000000000000')
    print('')

    print('')

    for i in range(lidar_num):#initialize time, xp and yp
        lidar_xyz[i][7] = now - 1.61605679*10**9
        lidar_xyz[i][10] = 1
        lidar_xyz[i][13] = 1
        lidar_xyz[i][16] = 1
        lidar_xyz[i][19] = 1

    if lidar_pub == []:#first time in code
        print("Start")
        for i in range(lidar_num):
            lidar_pub = np.append(lidar_pub, lidar_xyz[i], axis=0)
        lidar_pub = np.reshape(lidar_pub, (lidar_num,20))

    obj_id_array = []   #[lidar_pub[i][5] = [OBJ_ID]] Array
    pop_list = []

    lidar_pub = check(lidar_pub)

    for i in range(lidar_pub.shape[0]):
        obj_id_array.append(lidar_pub[i][5])
        print('obj_id', i, lidar_pub[i][5])

    print('obj_id_array', obj_id_array)


#########################################################################
###	update data from lidar_xyz


    for i in range(lidar_num):
        ## un-matched count
        for k in range(lidar_pub.shape[0]):
            lidar_pub[k][6] += 1

        K = 0 # check for dosen't match lidar_xyz[i]
        for j in range(lidar_pub.shape[0]):
            X = abs(lidar_xyz[i][0] - lidar_pub[j][0])
            Y = abs(lidar_xyz[i][1] - lidar_pub[j][1])
            if X < 4 and Y < 2:
                lidar_pub[j][0] = lidar_xyz[i][0]
                lidar_pub[j][1] = lidar_xyz[i][1]
                lidar_pub[j][2] = lidar_xyz[i][2]
                lidar_pub[j][3] = lidar_xyz[i][3]
                lidar_pub[j][4] = lidar_xyz[i][4]
                ##lidar_pub[j][5] is [OBJ_ID]
                lidar_pub[j][6] = 0  ## Initialize matched lidar_pub
                lidar_pub[j][7] = lidar_xyz[i][7]
                K = 1  # check
                break

        for n in range(lidar_pub.shape[0]):#delete unchanged data
            if lidar_pub[n][6] >= 10: # tolerance
                pop_list.append(n)
                print("pop processing", lidar_pub[n][5])
                print('pop processing', lidar_pub[n])
                lidar_pub[n] = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])

        if K == 0:  # When lidar_xyz[i] doesn't match with any lidar_pub's data
            while 1:
                random_id = random.randrange(1,100)
                if random_id not in obj_id_array:
                    lidar_xyz[i][5] = random_id
                    break

            lidar_pub = np.append(lidar_pub, lidar_xyz[i])
            lidar_pub = np.reshape(lidar_pub, (len(lidar_pub)/20, 20))
            lidar_pub[lidar_pub.shape[0]-1][10] = 1
            lidar_pub[lidar_pub.shape[0]-1][13] = 1
            lidar_pub[lidar_pub.shape[0]-1][16] = 1
            lidar_pub[lidar_pub.shape[0]-1][19] = 1
            obj_id_array.append(lidar_xyz[i][5]) ## New OBJ_ID

    for i in range(len(pop_list)): #delete trash
        pop_index = pop_list[i]
        lidar_pub = np.delete(lidar_pub, pop_index, axis = 0)


##################################################################################
#	 KALMAN filter
#	 0.x / 1.y / 2.z / 3.IMGx / 4.IMGy / 5.ID / 6.count / 7.time /
#	 8.xx / 9.xv / 10-13. xp1-4 / 14. yx / 15. yv / 16-19. yp1-4

    print('\033[95m')
    print('...............................')
    print('\033[0m')
    for i in range(lidar_pub.shape[0]):
        if lidar_pub[i][6] != 0:
            xx, xv, xP = predict(lidar_pub[i][0], 1, [[lidar_pub[i][10],lidar_pub[i][11]],[lidar_pub[i][12],lidar_pub[i][13]]],0.1)
            lidar_pub[i][8], lidar_pub[i][9] = xx, xv
            xp1, xp2, xp3, xp4 = xP[0][0], xP[0][1], xP[1][0], xP[1][1]
            lidar_pub[i][10], lidar_pub[i][11], lidar_pub[i][12], lidar_pub[i][13] =  xp1, xp2, xp3, xp4

            yx, yv, yP = predict(lidar_pub[i][1], 0, [[lidar_pub[i][10],lidar_pub[i][11]],[lidar_pub[i][12],lidar_pub[i][13]]],0.1)
            lidar_pub[i][14], lidar_pub[i][15] = yx, yv
            yp1, yp2, yp3, yp4 = yP[0][0], yP[0][1], yP[1][0], yP[1][1]
            lidar_pub[i][16], lidar_pub[i][17], lidar_pub[i][18], lidar_pub[i][19] =  yp1, yp2, yp3, yp4


        else:
            xx, xv, xP = update(lidar_pub[i][8],lidar_pub[i][9],[[lidar_pub[i][10],lidar_pub[i][11]],[lidar_pub[i][12],lidar_pub[i][13]]],lidar_pub[i][0])
            lidar_pub[i][8], lidar_pub[i][9] = xx, xv
            xp1, xp2, xp3, xp4 = xP[0][0], xP[0][1], xP[1][0], xP[1][1]
            lidar_pub[i][10], lidar_pub[i][11], lidar_pub[i][12], lidar_pub[i][13] =  xp1, xp2, xp3, xp4

            

##################################################################################
###	draw with opencv


    bridge = CvBridge()
    img = np.zeros((1500,1000,3), np.uint8)
    for i in range (70):
        if i % 2 is not 0:
            img = cv2.line(img, (430,i*30), (430,((i+1)*30)), (255,255,255),2)
            img = cv2.line(img, (570,i*30), (570,((i+1)*30)), (255,255,255),2)

    A = 500 - int(0) * 15
    B = 1300 - int(0) * 10 
    img = cv2.rectangle(img, (A,B), (A,B), (255,255,255), 50)
    for i in range(lidar_pub.shape[0]):
        X = 500 - int(lidar_pub[i][1]) * 40
        Y = 1300 - int(lidar_pub[i][0]) * 13

        if lidar_pub[i][5] % 5 == 0:
            img = cv2.rectangle(img, (X, Y), (X, Y), (255,0,0), 40)
        elif lidar_pub[i][5] % 5 == 1:
            img = cv2.rectangle(img, (X, Y), (X, Y), (0,255,0), 40)
        elif lidar_pub[i][5] % 5 == 2:
            img = cv2.rectangle(img, (X, Y), (X, Y), (0,0,255), 40)
        elif lidar_pub[i][5] % 5 == 3:
            img = cv2.rectangle(img, (X, Y), (X, Y), (255,0,255), 40)
        elif lidar_pub[i][5] % 5 == 4:
            img = cv2.rectangle(img, (X, Y), (X, Y), (0,255,255), 40)

    image_message = bridge.cv2_to_imgmsg(img, "8UC3")
    pub2.publish(image_message)
    pub.publish(lidar_pub.reshape(-1))



##################################################################################
###	initialize ros node

if __name__ == "__main__":  ### initialize ros node

    rospy.init_node('obj_tracking')

    pub = rospy.Publisher('/point_match/lidar_ID', data_array, queue_size=20)
    pub2 = rospy.Publisher('/point_match/tracking_image', Image, queue_size=20)
    pub3 = rospy.Publisher('/point_match/tracked_mark', MarkerArray, queue_size=1)

    sub = rospy.Subscriber('/point_match/lidar_array', data_array, callback_0)
    rospy.spin()
