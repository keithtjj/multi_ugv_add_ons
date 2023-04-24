#!/usr/bin/env python

import rospy 
from sensor_msgs.msg import Image 
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped, PoseStamped, Pose
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 
import numpy as np
from yolov7_ros.msg import Detection, Detections
from random import randint

pub_poi = rospy.Publisher('/poi_out', PoseStamped, queue_size=10)

bridge = CvBridge()
engage = False
current_pose = Pose()
poi_list = []
yolo_dets = []

def callback(data):
    raw = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
    pub_poi.publish(PoseStamped(header=Header(stamp=rospy.Time.now(),frame_id='test'), pose=current_pose))
    # detect people in the image
    for d in yolo_dets:
        b = d.bbox
        colour=(randint(0,255), randint(0,255), randint(0,255))
        cv2.rectangle(raw, (b[0], b[1]), (b[2], b[3]), colour, 1)
        center = int((b[0]+b[2])/2), int((b[1]+b[3])/2)

    #find doors
    lower_b = np.array([0,100,100])
    upper_b = np.array([0,130,130])
    mask = cv2.inRange(raw, lower_b, upper_b)
    cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in cnts[0]:
        cv2.drawContours(raw, [c], -1, (0, 255, 0), 2)
        area = cv2.contourArea(c)
        #rospy.loginfo(area)
        if area > 8000 and compare_pose(4.5):
            pub_poi.publish(PoseStamped(header=Header(stamp=rospy.Time.now(),frame_id='door'), pose=current_pose))
            poi_list.append(current_pose)
            rospy.loginfo('door')
    raw = cv2.resize(raw, (0,0), fx=2,fy=2)
    cv2.imshow("hunter", raw)
    cv2.waitKey(1)

def process_detects(detects):
    global poi_list, yolo_dets
    yolo_dets = []
    for d in detects.dets:
        yolo_dets.append(d.bbox)
        if d.name != 'person':
            continue
        b = d.bbox
        center = int((b[0]+b[2])/2), int((b[1]+b[3])/2)
        if compare_pose(5):
            poi_list.append(current_pose)
            rospy.loginfo(d.name)
            pub_poi.publish(PoseStamped(header=Header(stamp=rospy.Time.now(),frame_id=d.name), pose=current_pose))

def compare_pose(r):
    global current_pose
    for po in poi_list:
        dx = current_pose.position.x - po.position.x
        dy = current_pose.position.y - po.position.y
        dxy = dx**2 + dy**2
        #print(dxy)
        if dxy < r**2:
            return False
    rospy.loginfo(len(poi_list))
    return True

def get_pos(data):
    global current_pose 
    current_pose = data.pose.pose
    #rospy.loginfo(current_pose)
  
if __name__ == '__main__':
    rospy.init_node('hunter')
    rospy.Subscriber('/camera/image', Image, callback, queue_size=1, buff_size=2**24)
    rospy.Subscriber('/state_estimation', Odometry, get_pos)
    rospy.Subscriber('/detections', Detections, process_detects)    
    rospy.spin()
    cv2.destroyAllWindows()
