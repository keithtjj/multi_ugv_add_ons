import rospy
import rosbag
from std_msgs.msg import Int32MultiArray, String, Bool, Header
from geometry_msgs.msg import PointStamped, Point
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import DeleteModel
from pathlib import Path
import cv2
from tare_msgs.msg import NodeAndEdge

pub_wp = rospy.Publisher('/way_point', PointStamped, queue_size=1)
pub_gp = rospy.Publisher('/goal_point', PointStamped, queue_size=1)
pub_poi = rospy.Publisher('/poi_out', PointStamped, queue_size=10)
pub_keypose = rospy.Publisher('/sensor_coverage_planner/tare_planner_node/new_keypose', NodeAndEdge, queue_size=5)
pub_start = rospy.Publisher('/start_exploration', Bool, queue_size=5)

tare_mode = True
deleted = []

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

def wp_rebro(data):
    #rospy.loginfo("Received point at time %d", data.header.stamp.to_sec())
    if tare_mode:
        tare_wp = data
        pub_wp.publish(tare_wp)

def tare_switch(tog):
    global tare_mode
    tare_mode = tog.data

def goal_cb(msg):
    global tare_mode
    tare_mode = False
    msg.header.stamp = rospy.Time.now()
    pub_gp(msg)

def reach_status_cb(msg):
    if msg.data:
        global tare_mode
        tare_mode = True

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('saver')
    rospy.wait_for_service('/gazebo/delete_model')
    del_model_proxy = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    while not rospy.is_shutdown():
        cv2.imshow('waiting...', start_screen)
        k = cv2.waitKey(1)
        #print(k)
        if k == 27:
            cv2.destroyWindow('waiting...')
            poi = PointStamped(header=Header(stamp=rospy.Time.now(),frame_id='test'), point=(Point(0,0,0)))
            pub_poi.publish(poi)
            noe = NodeAndEdge(node_ind=0,keypose_id=0)
            pub_keypose.publish(noe)
            pub_start.publish(Bool(True))
            break

    rospy.Subscriber("/tare_way_point", PointStamped, wp_rebro)
    rospy.Subscriber("/far_way_point", PointStamped, wp_rebro)
    rospy.Subscriber("//sensor_coverage_planner/tare_planner_node/goal_point", PointStamped, goal_cb)
    rospy.Subscriber('/far_reach_goal_status', Bool, reach_status_cb)
    rospy.Subscriber('/toggle_wp', Bool, tare_switch) 
    rospy.Subscriber('/del_model_in', String, del_model)
    rospy.spin()
