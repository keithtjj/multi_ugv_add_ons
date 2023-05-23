import rospy
import rosbag
from std_msgs.msg import Int32MultiArray, String, Bool, Header
from geometry_msgs.msg import PointStamped, Pose, PoseStamped
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import DeleteModel
from pathlib import Path
import cv2
from tare_msgs.msg import NodeAndEdge

pub_wp = rospy.Publisher('/way_point', PointStamped, queue_size=1)
pub_poi = rospy.Publisher('/poi_out', PoseStamped, queue_size=10)
pub_keypose = rospy.Publisher('/sensor_coverage_planner/tare_planner_node/new_keypose', NodeAndEdge, queue_size=5)

tare_mode = True
current_pose = Pose()
deleted = []
door_list = [['door1', 43, -9], ['door2', 85, -4], ['door4', 56, 57], ['door3', 10, -2]]

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

def get_pos(data):
    global current_pose 
    current_pose = data.pose.pose

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
            poi = PoseStamped(header=Header(stamp=rospy.Time.now(),frame_id='test'), pose=current_pose)
            pub_poi.publish(poi)
            noe = NodeAndEdge(node_ind=0,keypose_id=0)
            pub_keypose.publish(noe)
            #pub_refresh_mqtt.publish('refresh')
            break

    rospy.Subscriber("/tare_way_point", PointStamped, wp_rebro)
    rospy.Subscriber('/state_estimation', Odometry, get_pos)
    rospy.Subscriber('/toggle_wp', Bool, tare_switch) 
    rospy.Subscriber('/del_model_in', String, del_model)
    rospy.spin()
