#!/usr/bin/env python

import rospy 
import rosbag
from std_msgs.msg import Header, String, Bool
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PointStamped, Point, PoseStamped

pub_gp = rospy.Publisher('/goal_point', PointStamped, queue_size=5)
pub_wp = rospy.Publisher('/way_point', PointStamped, queue_size=5)
pub_arrival = rospy.Publisher('/arrival', String, queue_size=5)
pub_joy = rospy.Publisher('/joy', Joy, queue_size=5)

poi_focus = 'door' #available poi types are: door, person

poi_pose_list=[]
poi_wp_list=[]
engage = False
waiting = True
n=1

start = Joy()
start.axes = [0,0,-1.0,0,1.0,1.0,0,0]
start.buttons = [0,0,0,0,0,0,0,1,0,0,0]
start.header.frame_id = "teleop_panel"

stop = Joy()
stop.axes = [0,0,0,0,0,0,0,0]
stop.buttons = [1,0,0,0,0,0,0,0,0,0,0]
stop.header.frame_id = "teleop_panel"

def set_engage(bool):
    global engage
    engage = bool.data

def save_poi(msg):
    global poi_pose_list, poi_wp_list
    if msg.header.frame_id != poi_focus:
        return
    #rospy.loginfo("received poi")
    if not (msg.pose in poi_pose_list):
        poi_pose_list.append(msg.pose)
        wp = PointStamped(header=msg.header, point = msg.pose.position)
        poi_wp_list.append(wp)
        rospy.loginfo("added new poi")

def update_goal_status(msg):
    global n, engage, poi_wp_list, stop
    if msg.data == True and len(poi_wp_list) != 0 and not engage:    
        engage = True
        stop.header.stamp = rospy.Time.now()
        pub_joy.publish(stop)
        pub_arrival.publish(String(poi_wp_list[0].header.frame_id))
        poi_wp_list.remove(poi_wp_list[0])
        rospy.loginfo('arrived at poi '+str(n))
        n+=1

if __name__ == '__main__':
    rospy.init_node('navi')
    rospy.sleep(1)
    rate = rospy.Rate(10)  
    rospy.loginfo('ready, waiting for pois')
    while not rospy.is_shutdown():
        rospy.Subscriber('/engaged', Bool, set_engage)
        rospy.Subscriber('/poi_in', PoseStamped, save_poi)
        rospy.Subscriber('/far_reach_goal_status', Bool, update_goal_status)
        
        if engage:
            continue
        else:
            start.header.stamp = rospy.Time.now()
            pub_joy.publish(start)

        if len(poi_wp_list) == 0 and not waiting:
            pub_gp.publish(PointStamped(header=Header(stamp=rospy.Time.now(),frame_id='map'), point=Point(0,0,0)))
            rospy.loginfo('going home, waiting for new pois')
            waiting = True

        elif len(poi_wp_list)>0:
            pub_gp.publish(PointStamped(header=Header(stamp=rospy.Time.now(),frame_id='map'), point=poi_wp_list[0].point))
            rospy.loginfo('going to poi '+str(n))
            waiting = False

        rate.sleep()
