#!/usr/bin/env python

import rospy 
import rosbag
from std_msgs.msg import Header, String, Bool
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PointStamped, Point, PoseStamped
from gazebo_msgs.srv import DeleteModel
from pathlib import Path
import cv2

pub_gp = rospy.Publisher('/goal_point', PointStamped, queue_size=5)
pub_wp = rospy.Publisher('/way_point', PointStamped, queue_size=5)
pub_arrival = rospy.Publisher('/arrival', String, queue_size=5)
pub_joy = rospy.Publisher('/joy', Joy, queue_size=5)
pub_refresh_mqtt = rospy.Publisher('/refresh_mqtt', String, queue_size=5)

poi_focus = 'door' #available poi types are: door, person

poi_pose_list=[]
poi_wp_list=[]
deleted = []
engage = False
waiting = True
tare_mode = True
n=1

start = Joy()
start.axes = [0,0,-1.0,0,1.0,1.0,0,0]
start.buttons = [0,0,0,0,0,0,0,1,0,0,0]
start.header.frame_id = "teleop_panel"

stop = Joy()
stop.axes = [0,1,0,0,0,0,0,0]
stop.buttons = [0,0,0,0,0,0,0,1,0,0,0]
stop.header.frame_id = "teleop_panel"

script_dir = Path( __file__ ).parent.absolute()
start_path = str(script_dir.joinpath('start.png'))
start_screen = cv2.imread(start_path)

def del_model(model):
    model_name = model.data
    if model_name in deleted:
        return
    del_model_proxy(model_name)
    deleted.append(model_name)
    rospy.loginfo('destroyed' + model_name)
    return

def set_engage(bool):
    global engage
    engage = bool.data

def save_poi(msg):
    global poi_pose_list, poi_wp_list
    if msg.header.frame_id == 'test':
        print('test poi')
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

def tare_switch(tog):
    global tare_mode
    tare_mode = tog.data

def wp_rebro(data):
    #rospy.loginfo("Received point at time %d", data.header.stamp.to_sec())
    if tare_mode:
        tare_wp = data
        pub_wp.publish(tare_wp)

if __name__ == '__main__':
    rospy.init_node('navi')
    rate = rospy.Rate(10)
    rospy.wait_for_service('/gazebo/delete_model')
    del_model_proxy = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    while not rospy.is_shutdown():
        cv2.imshow('waiting...', start_screen)
        k = cv2.waitKey(0)
        print(k)
        if k == 27:
            cv2.destroyWindow('waiting...')
            pub_refresh_mqtt.publish('refresh')
            break  

    rospy.loginfo('ready, waiting for pois')
    while not rospy.is_shutdown():
        rospy.Subscriber('/engaged', Bool, set_engage)
        rospy.Subscriber('/poi_in', PoseStamped, save_poi)
        rospy.Subscriber('/far_reach_goal_status', Bool, update_goal_status)
        rospy.Subscriber('/del_model_in', String, del_model)
        rospy.Subscriber('/toggle_wp', Bool, tare_switch)
        rospy.Subscriber('/far_way_point', PointStamped, wp_rebro) 

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
