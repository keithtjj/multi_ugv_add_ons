import rospy
import rosbag
from std_msgs.msg import String, Bool, Header
from geometry_msgs.msg import PointStamped, Point, PoseStamped
from gazebo_msgs.srv import DeleteModel
from pathlib import Path
import cv2
import message_filters

pub_wp = rospy.Publisher('/way_point', PointStamped, queue_size=1)
pub_gp = rospy.Publisher('/goal_point', PointStamped, queue_size=1)
pub_poi = rospy.Publisher('/poi_out', PointStamped, queue_size=10)
pub_start = rospy.Publisher('/start_exploration', Bool, queue_size=5)

tare_mode = True
far_mode = False
deleted = []
current_point = Point()

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

def twp_rebro(data):
    if tare_mode:
        tare_wp = data
        pub_wp.publish(tare_wp)

def fwp_rebro(data):
    if far_mode:
        far_wp = data
        pub_wp.publish(far_wp)

def tare_switch(tog):
    global tare_mode
    tare_mode = tog.data

def tsp_callback(tsp_msg, unreach_msg):
    global tare_mode, far_mode
    if tsp_msg.point == unreach_msg.point:
        far_mode = True
        tare_mode = False

def reach_status_cb(msg):
    if msg.data:
        global tare_mode
        tare_mode = True

def get_sq_dist(p1,p2):
    return (p2.x-p1.x)**2+(p2.y-p1.y)**2+(p2.z-p1.z)**2

def save_pose(msg):
    global current_point
    current_point = msg.pose.position

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
            pub_start.publish(Bool(True))
            break

    rospy.Subscriber("/tare_way_point", PointStamped, twp_rebro)
    rospy.Subscriber("/far_way_point", PointStamped, fwp_rebro)
    rospy.Subscriber("/pose_stamp", PoseStamped, save_pose)
    tsp_sub = message_filters.Subscriber("/sensor_coverage_planner/tare_planner_node/tsp_next", PointStamped)
    unreach_sub = message_filters.Subscriber('/sensor_coverage_planner/tare_planner_node/unreachable', PointStamped)
    ts = message_filters.TimeSynchronizer([tsp_sub, unreach_sub], 10)
    #ts.registerCallback(tsp_callback)
    rospy.Subscriber('/toggle_wp', Bool, tare_switch) 
    rospy.Subscriber('/del_model_in', String, del_model)
    rospy.spin()
