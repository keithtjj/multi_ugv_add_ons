#!/usr/bin/env python

import rospy
from std_msgs.msg import Header, Bool, String
from geometry_msgs.msg import TwistStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from yolov7_ros.msg import Detection, Detections

pub_vel = rospy.Publisher('/cmd_vel', TwistStamped, queue_size=5)
pub_engaged = rospy.Publisher('/engaged', Bool, queue_size=5)
pub_kill = rospy.Publisher('/del_model_out', String, queue_size=5)

door_list = [['door1', 43, -9], ['door2', 85, -4], ['door4', 56, 57], ['door3', 10, -2]]
engage = False
arrival = False
rawX = 320
last_seen = 0

def get_pos(data):
    global current_pose 
    current_pose = data.pose.pose

def del_model_sel(m):
    d = 2**8
    model_list = []
    if m == 'door':
        model_list = door_list
    for mo in model_list:
        dx = current_pose.position.x - mo[1]
        dy = current_pose.position.y - mo[2]
        dxy = dx**2 + dy**2
        if dxy < d:
            d = dxy
            m = mo[0]
    print('del ' + m)
    pub_kill.publish(String(m))

def process_detects(detects):
    global arrival, last_seen
    if not arrival:
        return
    targets = []
    for d in detects.dets:
        if d.name == poi_type:
            targets.append(d.bbox)
    lin_vel = 0
    ang_vel = 10
    for (x1, y1, x2, y2) in targets:
        if len(targets) == 1:
            centerX = int((x1+x2)/2)
            #rospy.loginfo('identified')
            if poi_type == 'door':
                if rawX/3 < centerX < rawX*2/3:
                    del_model_sel('door')
                    pub_engaged.publish(Bool(False))
                    arrival = False
                else:
                    ang_vel = 10 * (1-2*centerX/rawX)
                    area = (x1-x2)*(y2-y1)
                    lin_vel = 2 * (1-area/7000)
            elif poi_type == 'person':
                h = y2-y1
                ang_vel = 10 * (1-2*centerX/rawX)
                lin_vel = 10 * (1-h/120)
            else:
                rospy.loginfo('unknown poi, idk what to do')
            last_seen = centerX
        else:
            rospy.loginfo('too many/none')
            if last_seen > rawX/2:
                ang_vel = -10
            elif last_seen < rawX/2:
                ang_vel = 10
    pub_vel.publish(TwistStamped(header=Header(stamp=rospy.Time.now(),frame_id='vehicle'),
                                twist=(Twist(linear = Vector3(lin_vel,0,0), angular = Vector3(0,0,ang_vel)))))

def arrived(target):
    global arrival, poi_type
    arrival = True
    poi_type = target.data

if __name__ == '__main__':
    rospy.init_node('killer')
    rospy.Subscriber('/arrival', String, arrived)
    rospy.Subscriber('/state_estimation', Odometry, get_pos)
    rospy.Subscriber('/detections', Detections, process_detects)
    rospy.spin()
