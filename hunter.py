#!/usr/bin/env python

import rospy 
from sensor_msgs.msg import Image 
from std_msgs.msg import Int8, Header
from geometry_msgs.msg import PointStamped, Point, PoseStamped, Pose
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 
import numpy as np
 
# initialize the HOG descriptor/person detector
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
pub_poi = rospy.Publisher('/poi_out', PoseStamped, queue_size=10)
bridge = CvBridge()
global current_pose, poi_pose
current_pose = Pose()
poi_list = []

def callback(data):
    global poi_pose
    poi = PoseStamped(header=Header(stamp=rospy.Time.now(),frame_id='test'), pose=current_pose)
    rawraw = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
    raw = cv2.resize(rawraw, (0,0), fx=2,fy=2)
    gray = cv2.cvtColor(raw, cv2.COLOR_BGR2GRAY)

    # detect people in the image
    boxes, weights = hog.detectMultiScale(gray, padding=(8, 8), winStride=(8,8), hitThreshold=1.5)
    print(weights)
    for (x, y, w, h) in boxes:
        # display the detected boxes in the colour picture
        cv2.rectangle(raw, (x, y), (x+w, y+h), (0, 255, 0), 2)
        #d = -1.21*np.log(0.00138*h-0.159)
        if compare_pose(2):
            poi = PoseStamped(header=Header(stamp=rospy.Time.now(),frame_id='person'), pose=current_pose)
            poi_list.append(current_pose)
            rospy.loginfo('man') 
    #rospy.loginfo('spotted ' + str(len(boxes)))
    
    #find doors
    lower_b = np.array([0,100,100])
    upper_b = np.array([0,130,130])
    mask = cv2.inRange(raw, lower_b, upper_b)
    cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in cnts[0]:
        cv2.drawContours(raw, [c], -1, (0, 255, 0), 3)
        area = cv2.contourArea(c)
        #rospy.loginfo(area)
        if area > 35000 and compare_pose(2):
            poi = PoseStamped(header=Header(stamp=rospy.Time.now(),frame_id='door'), pose=current_pose)
            poi_list.append(current_pose)
            rospy.loginfo('door')
    pub_poi.publish(poi)
    cv2.imshow("hunter", raw)
    cv2.waitKey(1)

def compare_pose(r):
    global current_pose
    for po in poi_list:
        dx = current_pose.position.x - po.position.x
        dy = current_pose.position.y - po.position.y
        if abs(dx) < r or abs(dy) < r:
            return False
    rospy.loginfo(len(poi_list))
    return True

def get_pos(data):
    global current_pose 
    current_pose = data.pose.pose
    #rospy.loginfo(current_pose)

def get_dist(a, b):
    return np.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)
  
if __name__ == '__main__':
    rospy.init_node('hunter')
    engage = False
    rospy.sleep(1)

    rospy.Subscriber('/camera/image', Image, callback)
    rospy.Subscriber('/state_estimation', Odometry, get_pos)
    rospy.spin()
    
    cv2.destroyAllWindows()
