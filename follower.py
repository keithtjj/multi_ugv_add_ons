#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image 
from std_msgs.msg import Header, Bool, String
from geometry_msgs.msg import TwistStamped, Twist, Vector3
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 
import numpy as np
from yolov7_ros.msg import Detection, Detections
from random import randint

pub_vel = rospy.Publisher('/cmd_vel', TwistStamped, queue_size=1)
pub_engaged = rospy.Publisher('/engaged', Bool, queue_size=5)
pub_tare_toggle = rospy.Publisher('/toggle_tare', Bool, queue_size=5)

bridge = CvBridge()
engage = False
tare_mode = True
arrival = False
persons = []
raw_x = 320

def detector(data):
    global engage, arrival
    raw = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
    # detect people in the image
    # returns the bounding boxes for the detected objects
    for b in persons:
        colour=(randint(0,255), randint(0,255), randint(0,255))
        cv2.rectangle(raw, (b[0], b[1]), (b[2], b[3]), colour, 1)
        center = int((b[0]+b[2])/2), int((b[1]+b[3])/2)

    raw = cv2.resize(raw, (0,0), fx=2,fy=2)
    cv2.imshow("follower", raw)
    cv2.waitKey(1)
    
def process_detects(detects):
    global persons
    persons = []
    for d in detects.dets:
        if d.name == 'person':
                persons.append(d.bbox)
    if not arrival:
        return
    
    lin_vel = 0
    ang_vel = 10
    if len(persons) == 1:
        global tare_mode
        if tare_mode == True:
            pub_tare_toggle.publish(Bool(False))
            tare_mode = False
            rospy.loginfo('tare broken')
            return
        for (x1, y1, x2, y2) in persons:
            #rospy.loginfo('identified')
            center_x = (x1+x2)/2
            h = y1-y2
            ang_vel = 30 * (1-2*center_x/raw_x)
            lin_vel = 30 * (1-h/90)

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
    rospy.Subscriber('/camera/image', Image, detector, queue_size=1, buff_size=2**24)
    rospy.Subscriber('/detections', Detections, process_detects)
    rospy.spin()
    cv2.destroyAllWindows()