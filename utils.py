#############################
#### Imports
#############################

# General 
import os
import cv2
import numpy as np
import math

# ROS 
try:
    import rospy
    from rospy.numpy_msg import numpy_msg
    from sensor_msgs.msg import LaserScan
    from sensor_msgs.msg import Image
    from ackermann_msgs.msg import AckermannDriveStamped
except:
    print('ROS is not installed')

# iPython Display
import PIL.Image
from io import BytesIO
import IPython.display
import time

# Used for HSV select
import threading
try:
    import ipywidgets as widgets
except:
    print('ipywidgets is not installed')

#############################
#### Racecar ROS Class
#############################

# Starter code class that handles the fancy stuff. No need to modify this! 
class Racecar:
    SCAN_TOPIC = "/scan"
    IMAGE_TOPIC = "/camera"
    DRIVE_TOPIC = "/drive"
    
    def __init__(self):
        self.sub_scan = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, callback=self.scan_callback)
        self.sub_image = rospy.Subscriber(self.IMAGE_TOPIC, Image, callback=self.image_callback)
        self.pub_drive = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=1)
        self.last_drive = AckermannDriveStamped()
    
    def image_callback(self, msg):
        self.last_image = msg.data
        
    def show_last_image(self):
        im = np.fromstring(self.last_image,dtype=np.uint8).reshape((480,-1,3))[...,::-1]
        return im
        
    def scan_callback(self, msg):
        self.last_scan = msg.ranges
        
    def drive(self, speed, angle):
        msg = AckermannDriveStamped()
        msg.drive.speed = speed
        msg.drive.steering_angle = math.degrees(angle)  # thresholded at 0.25 radians = 14 degrees
        self.last_drive = msg
    
    def stop(self):
        self.drive(0, 0) #self.last_drive.drive.steering_angle)
    
    def look(self):
        return self.last_image
    
    def scan(self):
        return self.last_scan
    
    def run(self, func, limit=10):
        r = rospy.Rate(60)
        t = rospy.get_time()
        cap = cv2.VideoCapture(2)
        while rospy.get_time() - t < time_limit and not rospy.is_shutdown():
            func(cap.read()[1])
            self.pub_drive.publish(self.last_drive)
            r.sleep()
        cap.release()
        print("END OF ROSPY RUN")
        self.stop()
        self.pub_drive.publish(self.last_drive)
        time.sleep(0.1)
        
        
#############################
#### Parameters
#############################

# Video Capture Port
video_port = 2

# Display ID
current_display_id = 1 # keeps track of display id


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

    
#############################
#### Identify Cone
#############################

# returns the contour with the biggest area from a list of contours
def find_greatest_contour(contours):
    largest_contour = [-1, -1] 
    for i, cnt in enumerate(contours):
        if (cv2.contourArea(cnt) >= largest_contour[1]):
            largest_contour  = [i, cv2.contourArea(cnt)]
    return contours[largest_contour[0]]

def show_video(func, time_limit, rc):
    global display, current_display_id
    display = IPython.display.display('', display_id=current_display_id)
    current_display_id += 1
    
    rc.run(func, time_limit)

def show_image(func):
    global display, current_display_id
    display = IPython.display.display('', display_id=current_display_id)
    current_display_id += 1
    
    cap = cv2.VideoCapture(video_port)
    frame = func(cap.read()[1])   
    show_frame(frame)
    cap.release()


#############################
#### Line Follow
#############################

def crop(img, top, bottom):
    return img[top:bottom,:,:]


#############################
#### HSV Select
#############################

# Mask and display video
def hsv_select_live(limit = 10, fps = 4):
    global current_display_id
    display = IPython.display.display('', display_id=current_display_id)
    current_display_id += 1
    
    # Create sliders
    h = widgets.IntRangeSlider(value=[0, 179], min=0, max=179, description='Hue:', continuous_update=True, layout=widgets.Layout(width='100%'))
    s = widgets.IntRangeSlider(value=[0, 255], min=0, max=255, description='Saturation:', continuous_update=True, layout=widgets.Layout(width='100%'))
    v = widgets.IntRangeSlider(value=[0, 255], min=0, max=255, description='Value:', continuous_update=True, layout=widgets.Layout(width='100%'))
    display.update(h)
    display.update(s)
    display.update(v)
    
    # Live masked video for the thread
    def show_masked_video():
        video = cv2.VideoCapture(video_port)
        start = time.time()
        while time.time() - start < limit:
            frame = video.read()[1]
            if frame is not None:
                hsv_min = (h.value[0], s.value[0], v.value[0])
                hsv_max = (h.value[1], s.value[1], v.value[1])
                frame = cv2.resize(frame, (320, 240))
                frame = cv2.flip(frame, 1)
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img_hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
                mask = cv2.inRange(img_hsv, hsv_min, hsv_max)
                img_masked = cv2.bitwise_and(frame, frame, mask = mask)
                f = BytesIO()
                PIL.Image.fromarray(img_masked).save(f, 'jpeg')
                img_jpeg = IPython.display.Image(data=f.getvalue())
                display.update(img_jpeg)
                time.sleep(1.0 / fps)
        video.release()
    
    # Open video on new thread (needed for slider update)
    hsv_thread = threading.Thread(target=show_masked_video)
    hsv_thread.start()