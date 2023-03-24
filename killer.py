#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image 
from std_msgs.msg import Int8, Header, Bool, String
from geometry_msgs.msg import TwistStamped, Twist, Vector3
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 
import numpy as np
 
# initialize the HOG descriptor/person detector
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
bridge = CvBridge()

engage = False

pub_vel = rospy.Publisher('/cmd_vel', TwistStamped, queue_size=5)
pub_engaged = rospy.Publisher('/engaged', Bool, queue_size=5)
pub_door = rospy.Publisher('/box_cmd_vel', Twist, queue_size=1)

def detector(data):
    global engage, arrival
    rawraw = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    raw = cv2.resize(cv2.cvtColor(rawraw, cv2.COLOR_BGR2RGB), (0,0), fx=2,fy=2)
    gray = cv2.cvtColor(raw, cv2.COLOR_BGR2GRAY)

    # detect people in the image
    # returns the bounding boxes for the detected objects
    boxes, weights = hog.detectMultiScale(gray, padding=(8, 8), winStride=(4,4))
    for (x, y, w, h) in boxes:
        # display the detected boxes in the colour picture
        cv2.rectangle(raw, (x, y), (x + w, y + h), (0, 255, 0), 2)
    
    #find doors
    (T, thresh) = cv2.threshold(gray, 5, 255, cv2.THRESH_BINARY_INV)
    cnts, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in cnts:
        cv2.drawContours(raw, [c], -1, (0, 255, 0), 3)

    cv2.imshow("killer", raw)
    cv2.waitKey(1)
    
    if not arrival:
        return
    
    if poi_type == 'door':
        for c in cnts:
            M = cv2.moments(c)
            centerX = int(M["m10"] / M["m00"])
            if raw.shape[1]/3 < centerX < raw.shape[1]*2/3:
                pub_door.publish(Twist(linear = Vector3(0.5,0,0), angular = Vector3(0,0,-0.5)))
                pub_engaged.publish(Bool(False))
            elif centerX < raw.shape[1]/2:
                pub_vel.publish(TwistStamped(header=Header(stamp=rospy.Time.now(),frame_id='vehicle'),
                                            twist=(Twist(linear = Vector3(0,0,0), angular = Vector3(0,0,1)))))
            elif centerX > raw.shape[1]/2:
                pub_vel.publish(TwistStamped(header=Header(stamp=rospy.Time.now(),frame_id='vehicle'),
                                            twist=(Twist(linear = Vector3(0,0,0), angular = Vector3(0,0,-1)))))

    if len(boxes) == 1:
        for (x, y, w, h) in boxes:
            centerX = x+w/2
            rospy.loginfo('identified')
            if engage == True:
                pub_vel.publish(TwistStamped(header=Header(stamp=rospy.Time.now(),frame_id='vehicle'),
                                            twist=(Twist(linear = Vector3(0,0,0), angular = Vector3(0,0,0)))))
            elif raw.shape[1]/3 < centerX < raw.shape[1]*2/3:
                pub_vel.publish(TwistStamped(header=Header(stamp=rospy.Time.now(),frame_id='vehicle'),
                                            twist=(Twist(linear = Vector3(0,0,0), angular = Vector3(0,0,0)))))
            
                #pub_vel.publish(TwistStamped(header=Header(stamp=rospy.Time.now(),frame_id='vehicle'),
                #                            twist=(Twist(linear = Vector3(5,0,0), angular = Vector3(0,0,0)))))
                engage = True
                rospy.loginfo('engage')

            elif centerX < raw.shape[1]/2:
                pub_vel.publish(TwistStamped(header=Header(stamp=rospy.Time.now(),frame_id='vehicle'),
                                            twist=(Twist(linear = Vector3(0,0,0), angular = Vector3(0,0,1)))))
            elif centerX > raw.shape[1]/2:
                pub_vel.publish(TwistStamped(header=Header(stamp=rospy.Time.now(),frame_id='vehicle'),
                                            twist=(Twist(linear = Vector3(0,0,0), angular = Vector3(0,0,-1)))))

    elif len(boxes)!=1 and engage==False:
            pub_vel.publish(TwistStamped(header=Header(stamp=rospy.Time.now(),frame_id='vehicle'),
                                                        twist=(Twist(linear = Vector3(0,0,0), angular = Vector3(0,0,3)))))
            rospy.loginfo('not found')
    elif len(boxes)==0 and engage==True:
        arrival = False
        engage = False
        rospy.loginfo('destroyed')
        pub_engaged.publish(Bool(False))
    
def arrived(target):
    global arrival, poi_type
    arrival = True
    poi_type = target.data

if __name__ == '__main__':
    global arrival
    arrival = False
    rospy.init_node('killer')
    rospy.Subscriber('/arrival', String, arrived)
    rospy.Subscriber('/camera/image', Image, detector)

    rospy.spin()
    cv2.destroyAllWindows()