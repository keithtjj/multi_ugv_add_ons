#!/usr/bin/env python

import rospy 
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Odometry
from yolov7_ros.msg import Detection, Detections
from random import randint

pub_poi = rospy.Publisher('/poi_out', PoseStamped, queue_size=10)

engage = False
current_pose = Pose()
poi_list = []

def process_detects(detects):
    global poi_list
    poi = PoseStamped(header=Header(stamp=rospy.Time.now(),frame_id='test'), pose=current_pose)
    for d in detects.dets:
        if compare_pose(5, d.name):
            poi = PoseStamped(header=Header(stamp=rospy.Time.now(),frame_id=d.name), pose=current_pose)
            poi_list.append(poi)
            rospy.loginfo(d.name)
            pub_poi.publish(poi)
    if len(detects.dets) == 0:
        pub_poi.publish(poi)

def compare_pose(r, name):
    global current_pose
    for po in poi_list:
        if po.header.frame_id != name:
            return
        dx = current_pose.position.x - po.pose.position.x
        dy = current_pose.position.y - po.pose.position.y
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
    rospy.Subscriber('/state_estimation', Odometry, get_pos)
    rospy.Subscriber('/detections', Detections, process_detects)    
    rospy.spin()