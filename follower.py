#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image 
from std_msgs.msg import Int8, Header, Bool
from geometry_msgs.msg import Pose, TwistStamped, Twist, Vector3
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 
import numpy as np
 
# initialize the HOG descriptor/person detector
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
bridge = CvBridge()

engage = False
tare_mode = True
pub_vel = rospy.Publisher('/cmd_vel', TwistStamped, queue_size=1)
pub_engaged = rospy.Publisher('/engaged', Bool, queue_size=5)
pub_tare_toggle = rospy.Publisher('/toggle_tare', Bool, queue_size=5)

def detector(data):
    global engage, arrival
    rawraw = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
    raw = cv2.resize(rawraw, (0,0), fx=2,fy=2)
    gray = cv2.cvtColor(raw, cv2.COLOR_BGR2GRAY)

    # detect people in the image
    # returns the bounding boxes for the detected objects
    boxes, weights = hog.detectMultiScale(gray, padding=(8, 8), winStride=(8,8))
    for (x, y, w, h) in boxes:
        # display the detected boxes in the colour picture
        cv2.rectangle(raw, (x, y), (x + w, y + h), (0, 255, 0), 2)
    cv2.imshow("follower", raw)
    cv2.waitKey(1)

    if len(boxes) == 1:
        global tare_mode
        if tare_mode == True:
            pub_tare_toggle.publish(Bool(False))
            tare_mode = False
            rospy.loginfo('tare broken')
            pub_vel.publish(TwistStamped(header=Header(stamp=rospy.Time.now(),frame_id='vehicle'),
                                        twist=(Twist(linear = Vector3(0,0,0), angular = Vector3(0,0,0)))))
            return
        for (x, y, w, h) in boxes:
            _, raw_x, _ = raw.shape

            rospy.loginfo('identified')

            ang_vel = 30 * (1-2*(x+w/2)/raw_x)
            if h < 190:
                lin_vel = 30 * (1-h/190)
            else: 
                lin_vel = 0
            pub_vel.publish(TwistStamped(header=Header(stamp=rospy.Time.now(),frame_id='vehicle'),
                                        twist=(Twist(linear = Vector3(lin_vel,0,0), angular = Vector3(0,0,ang_vel)))))
            
    else:
        pub_vel.publish(TwistStamped(header=Header(stamp=rospy.Time.now(),frame_id='vehicle'),
                                        twist=(Twist(linear = Vector3(0,0,0), angular = Vector3(0,0,0)))))

def arrived(bool):
    global arrival
    arrival = True

if __name__ == '__main__':
    global arrival
    arrival = False
    rospy.init_node('killer')
    rospy.Subscriber('/arrival', Bool, arrived)
    rospy.Subscriber('/camera/image', Image, detector)

    rospy.spin()
    cv2.destroyAllWindows()