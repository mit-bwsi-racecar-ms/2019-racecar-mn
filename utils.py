#############################
#### Imports
#############################

# General 
import os
import cv2
import numpy as np

# ROS 
import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped

# iPython Display
import PIL.Image
from io import BytesIO
import IPython.display
import time


#############################
#### General Display
#############################

def show_frame(frame):
    global display
    frame = cv2.resize(frame, (320, 240))
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    f = BytesIO()
    PIL.Image.fromarray(frame).save(f, 'jpeg')
    img = IPython.display.Image(data=f.getvalue())
    display.update(img)
    
last_time = time.time()
def staggered_video(frame, last_time):
    #print(last_time, time.time())
    if time.time() - last_time >= 1:
        show_frame(frame)
        last_time = time.time()

#############################
#### Identify Cone
#############################

def show_identified_image(colorRange, min_size, func):
    global display
    display = IPython.display.display('', display_id=1)
    cap = cv2.VideoCapture(1)
    frame = func(cap.read()[1], (colorRange, min_size))
    show_frame(frame)
    cap.release()

def show_identified_video(colorRange, min_size, func, rc):
    global display
    display = IPython.display.display('', display_id=1)
    cap = cv2.VideoCapture(1)
    rc.run(func, (colorRange, min_size), 10)
    cap.release()
    
def get_moment(contour):
    M = cv2.moments(contour)
    if M['m00'] != 0.0:
        cx = int(M['m10']/M['m00'])
        c = -.5
        if res[0] < 0:
            c = .5
        ratio = 0 
        dist = cx - screen_center*1.0
        if abs(dist) > 20 and screen_center != 0:
            ratio = dist/screen_center
            res[1] = ratio*c 
            #print(dist, ratio, angle)

#############################
#### Display
#############################

def find_greatest_contour(contours):
    largest_contour = [-1, -1] 
    for i, cnt in enumerate(contours):
        if (cv2.contourArea(cnt) >= largest_contour[1]):
            largest_contour  = [i, cv2.contourArea(cnt)]
    return contours[largest_contour[0]]