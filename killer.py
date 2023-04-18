#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image 
from std_msgs.msg import Header, Bool, String
from geometry_msgs.msg import TwistStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import DeleteModel
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 
import numpy as np

pub_vel = rospy.Publisher('/cmd_vel', TwistStamped, queue_size=5)
pub_engaged = rospy.Publisher('/engaged', Bool, queue_size=5) 

# initialize the HOG descriptor/person detector
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
bridge = CvBridge()

model_list = [['door1', 43, -9], ['door2', 85, -4], ['door4', 56, 57], ['door3', 10, -2]]
engage = False

def get_pos(data):
    global current_pose 
    current_pose = data.pose.pose

def del_model(model_name : str):
    rospy.wait_for_service('/gazebo/delete_model')
    del_model_proxy = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    del_model_proxy(model_name)
    rospy.loginfo('destroyed' + model_name)
    return

def del_model_sel():
    d = 255
    for mo in model_list:
        dx = current_pose.position.x - mo[1]
        dy = current_pose.position.y - mo[2]
        dxy = dx**2 + dy**2
        if dxy < d:
            d = dxy
            m = mo[0]
    print('del' + m)
    del_model(m)

def detector(data):
    global engage, arrival
    rawraw = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
    raw = cv2.resize(rawraw, (0,0), fx=2,fy=2)
    gray = cv2.cvtColor(raw, cv2.COLOR_BGR2GRAY)

    # detect people in the image
    # returns the bounding boxes for the detected objects
    boxes, weights = hog.detectMultiScale(gray, padding=(8, 8), winStride=(4,4), hitThreshold=1.5)
    for (x, y, w, h) in boxes:
        # display the detected boxes in the colour picture
        cv2.rectangle(raw, (x, y), (x + w, y + h), (0, 255, 0), 2)
    
    #find doors
    #(T, thresh) = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
    lower_b = np.array([0,100,100])
    upper_b = np.array([0,130,130])
    mask = cv2.inRange(raw, lower_b, upper_b)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in cnts:
        cv2.drawContours(raw, [c], -1, (0, 255, 0), 3)

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
                del_model_sel()
                pub_engaged.publish(Bool(False))
                rospy.loginfo('destroyed')
                arrival = False
            else:
                ang_vel = 10 * (1-2*centerX/raw_x)
        if len(cnts)==0:
            ang_vel = 10

    elif poi_type == 'person':
        if len(boxes) == 1:
            for (x, y, w, h) in boxes:
                centerX = x+w/2
                rospy.loginfo('identified')
                if raw.shape[1]/3 < centerX < raw.shape[1]*2/3:
                    rospy.loginfo('engage')
                    del_model('person_standing')
                    pub_engaged.publish(Bool(False))
                    rospy.loginfo('unalived')
                    arrival = False
                else:
                    rospy.loginfo('aiming')
                    ang_vel = 30 * (1-2*(x+w/2)/raw_x)
                    if h < 190:
                        lin_vel = 10 * (1-h/190)
                    
        else:
            ang_vel = 10
            rospy.loginfo('not found')
    
    pub_vel.publish(TwistStamped(header=Header(stamp=rospy.Time.now(),frame_id='vehicle'),
                                twist=(Twist(linear = Vector3(lin_vel,0,0), angular = Vector3(0,0,ang_vel)))))

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
    rospy.Subscriber('/state_estimation', Odometry, get_pos)

    rospy.spin()
    cv2.destroyAllWindows()