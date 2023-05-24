#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image 
from std_msgs.msg import Header, Bool, String, Int8
from geometry_msgs.msg import TwistStamped, Twist, Vector3
import cv2 
import numpy as np
from yolov7_ros.msg import Detection, Detections
from random import randint

pub_vel = rospy.Publisher('/cmd_vel', TwistStamped, queue_size=1)
pub_engaged = rospy.Publisher('/engaged', Bool, queue_size=5)
pub_tare_toggle = rospy.Publisher('/toggle_tare', Bool, queue_size=5)
pub_stop = rospy.Publisher('/stop', Int8, queue_size=5)


engage = False
tare_mode = True
arrival = False
persons = []
raw_x = 320
    
def process_detects(detects):
    global persons
    persons = []
    for d in detects.dets:
        if d.name == 'person':
            persons.append(d.bbox)
    
    lin_vel = 0
    ang_vel = 10
    if len(persons) == 1:
        pub_stop.publish(Int8(2))
        global tare_mode
        if tare_mode == True:
            pub_tare_toggle.publish(Bool(False))
            tare_mode = False
            rospy.loginfo('tare broken')
            return
        for (x1, y1, x2, y2) in persons:
            #rospy.loginfo('identified')
            center_x = (x1+x2)/2
            h = y2-y1
            ang_vel = 10 * (1-2*center_x/raw_x)
            lin_vel = 10 * (1-h/90)

        pub_vel.publish(TwistStamped(header=Header(stamp=rospy.Time.now(),frame_id='vehicle'),
                                    twist=(Twist(linear = Vector3(lin_vel,0,0), angular = Vector3(0,0,ang_vel)))))

def arrived(msg):
    global arrival
    if msg.data == 'person':
        arrival = True
    else:
        arrival = False

if __name__ == '__main__':
    rospy.init_node('follower')
    rospy.Subscriber('/arrival', String, arrived)
    rospy.Subscriber('/detections', Detections, process_detects)
    rospy.spin()
    cv2.destroyAllWindows()