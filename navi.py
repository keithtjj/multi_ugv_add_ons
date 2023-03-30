#!/usr/bin/env python

import rospy 
import rosbag
from std_msgs.msg import Int8, Header, String, Bool
from geometry_msgs.msg import PointStamped, Point, Pose, TwistStamped
from nav_msgs.msg import Odometry

from pathlib import Path
import numpy as np

pub_gp = rospy.Publisher('/goal_point', PointStamped, queue_size=5)
pub_wp = rospy.Publisher('/way_point', PointStamped, queue_size=5)
pub_vel = rospy.Publisher('/cmd_vel', TwistStamped, queue_size=1)
pub_read_vg = rospy.Publisher('/read_file_dir', String, queue_size=5)
pub_arrival = rospy.Publisher('/arrival', String, queue_size=5)

script_dir = Path( __file__ ).parent.absolute()
ws = script_dir.parent.parent.parent
folder = ws.joinpath('store')
map_name = 'map.vgh'
bag_name = 'spaces.bag'

old_bag = rosbag.Bag(folder.joinpath(bag_name))
tare_mode = True

def get_poi_from_bag(b):
    t=0
    poi_pose_list=[]
    poi_wp_list=[]
    for topic,msg,time in b.read_messages('/poi'):
        if not (msg.pose in poi_pose_list):
            poi_pose_list.append(msg.pose)
            wp = PointStamped(header=msg.header, point = msg.pose.position)
            poi_wp_list.append(wp)
    return poi_pose_list, poi_wp_list

def get_pos(data):
    global current_pose 
    current_pose = data.pose.pose
    #rospy.loginfo(current_pose)

def vel_rebro(data):
    #rospy.loginfo("Received point at time %d", data.header.stamp.to_sec())
    global tare_mode
    if tare_mode:
        aede_vel = data
        pub_vel.publish(aede_vel)
    return

def points_in_range(pt1, pt2, d):
    dx = pt1.x - pt2.x
    dy = pt1.y - pt2.y
    if abs(dx) < d and abs(dy) < d:
        return True
    else:
        return False  

def set_engage(bool):
    global engage
    engage = False

if __name__ == '__main__':
    rospy.init_node('navi')
    rospy.sleep(1)

    global current_pose, engage
    engage = False
    current_pose = Pose()
    _, poi_wp_list = get_poi_from_bag(old_bag)
    pub_read_vg.publish(String(str(folder.joinpath(map_name))))
    n=1
    while not rospy.is_shutdown():
        rospy.Subscriber('/state_estimation', Odometry, get_pos)
        rospy.Subscriber('/engaged', Bool, set_engage)
        rospy.Subscriber("/aede_cmd_vel", TwistStamped, vel_rebro)

        if engage:
            continue

        if len(poi_wp_list) == 0:
            pub_gp.publish(PointStamped(header=Header(stamp=rospy.Time.now(),frame_id='map'), point=Point(0,0,0)))
            rospy.loginfo('No more pois')
        elif points_in_range(current_pose.position, poi_wp_list[0].point, 0.5):
            engage = True
            pub_arrival.publish(String(poi_wp_list[0].header.frame_id))
            poi_wp_list.remove(poi_wp_list[0])
            rospy.loginfo('arrived at poi '+str(n))
            n+=1
        else:
            pub_gp.publish(PointStamped(header=Header(stamp=rospy.Time.now(),frame_id='map'), point=poi_wp_list[0].point))
            rospy.loginfo('going to poi '+str(n))

        rospy.sleep(1)