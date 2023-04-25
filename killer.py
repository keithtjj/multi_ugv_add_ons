#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image 
from std_msgs.msg import Header, Bool, String
from geometry_msgs.msg import TwistStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 
import numpy as np
from yolov7_ros.msg import Detection, Detections
from random import randint

pub_vel = rospy.Publisher('/cmd_vel', TwistStamped, queue_size=5)
pub_engaged = rospy.Publisher('/engaged', Bool, queue_size=5)
pub_kill = rospy.Publisher('/del_model_out', String, queue_size=5)

bridge = CvBridge()
door_list = [['door1', 43, -9], ['door2', 85, -4], ['door4', 56, 57], ['door3', 10, -2]]
engage = False
yolo_dets = []
persons = []

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
    print('del' + m)
    pub_kill.publish(String(m))

def detector(data):
    global engage, arrival
    raw = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
    pub_kill.publish(String('test'))

    # detect people in the image
    # returns the bounding boxes for the detected objects
    for d in yolo_dets:
        colour=(randint(0,255), randint(0,255), randint(0,255))
        b = d.bbox
        cv2.rectangle(raw, (b[0], b[1]), (b[2], b[3]), colour, 1)
        center = int((b[0]+b[2])/2), int((b[1]+b[3])/2)
    
    #find doors
    #(T, thresh) = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
    lower_b = np.array([0,100,100])
    upper_b = np.array([0,130,130])
    mask = cv2.inRange(raw, lower_b, upper_b)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in cnts:
        cv2.drawContours(raw, [c], -1, (0, 255, 0), 2)
    raw = cv2.resize(raw, (0,0), fx=2,fy=2)
    cv2.imshow("killer", raw)
    cv2.waitKey(1)
    
    if not arrival:
        return
    
    lin_vel = 0
    ang_vel = 0
    _, raw_x, _ = raw.shape
    if poi_type == 'door':
        for c in cnts:
            M = cv2.moments(c)
            centerX = int(M["m10"] / (M["m00"]+1))
            if raw_x/3 < centerX < raw_x*2/3 and M["m00"] > 20000:
                del_model_sel('door')
                pub_engaged.publish(Bool(False))
                rospy.loginfo('destroyed')
                arrival = False
            else:
                ang_vel = 10 * (1-2*centerX/raw_x)
        if len(cnts)==0:
            ang_vel = 10

    elif poi_type == 'person':
        if len(persons) == 1:
            for (x1, y1, x2, y2) in persons:
                center = int((x1+x2)/2), int((y1+y2)/2)
                rospy.loginfo('identified')
                if raw.shape[1]/3 < centerX < raw.shape[1]*2/3:
                    rospy.loginfo('engage')
                    pub_kill.publish(String('person'))
                    pub_engaged.publish(Bool(False))
                    rospy.loginfo('unalived')
                    arrival = False
                else:
                    rospy.loginfo('aiming')
                    ang_vel = 10 * (1-2*center[0]/raw_x)
                    h = y2-y1
                    if h < 90:
                        lin_vel = 10 * (1-h/90)
                    
        else:
            ang_vel = 10
            rospy.loginfo('not found')
    
    pub_vel.publish(TwistStamped(header=Header(stamp=rospy.Time.now(),frame_id='vehicle'),
                                twist=(Twist(linear = Vector3(lin_vel,0,0), angular = Vector3(0,0,ang_vel)))))

def process_detects(detects):
    global yolo_dets, persons
    yolo_dets = []
    persons = []
    for d in detects.dets:
        yolo_dets.append(d)
        if d.name == 'person':
                persons.append(d.bbox)

def arrived(target):
    global arrival, poi_type
    arrival = True
    poi_type = target.data

if __name__ == '__main__':
    global arrival
    arrival = False
    rospy.init_node('killer')
    rospy.Subscriber('/arrival', String, arrived)
    rospy.Subscriber('/camera/image', Image, detector, queue_size=1, buff_size=2**24)
    rospy.Subscriber('/state_estimation', Odometry, get_pos)
    rospy.Subscriber('/detections', Detections, process_detects)
    rospy.spin()
    cv2.destroyAllWindows()