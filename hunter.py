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
yolo_dets = []

def process_detects(detects):
    global poi_list
    for d in detects.dets:
        yolo_dets.append(d)
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
    rospy.Subscriber('/state_estimation', Odometry, get_pos)
    rospy.Subscriber('/detections', Detections, process_detects)    
    rospy.spin()