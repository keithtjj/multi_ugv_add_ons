import rospy
import rosbag
from std_msgs.msg import Int32MultiArray, String, Bool, Header
from geometry_msgs.msg import PointStamped, Point, PoseStamped, TwistStamped
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

from pathlib import Path
import cv2
import numpy as np

script_dir = Path( __file__ ).parent.absolute()
ws = script_dir.parent.parent.parent
folder = ws.joinpath('store')
bag_name = 'spaces.bag'
map_name = 'map.vgh'

try:
    old_bag = rosbag.Bag(folder.joinpath(bag_name))
except: pass

pub_covered = rospy.Publisher('/Combined_Covered_Indices', Int32MultiArray, queue_size=1)
pub_exploring = rospy.Publisher('/Combined_Exploring_Indices', Int32MultiArray, queue_size=1)
pub_wp = rospy.Publisher('/way_point', PointStamped, queue_size=1)
pub_vel = rospy.Publisher('/cmd_vel', TwistStamped, queue_size=1)
pub_save_vg = rospy.Publisher('/save_file_dir', String, queue_size=5)
pub_read_vg = rospy.Publisher('/read_file_dir', String, queue_size=5)

tare_mode = True

def save_covered(array):
    bag.write('/Combined_Covered_Indices', array)
    rospy.loginfo('saved c')
    pub_save_vg.publish(String(str(folder.joinpath(map_name))))
    rospy.loginfo('saved map')
    return 

def save_exploring(array):
    bag.write('/Combined_Exploring_Indices', array)
    rospy.loginfo('saved e')
    return 

def get_latest_covered_from_bag(b):
    t=0
    covered_array = Int32MultiArray()
    for topic,msg,time in b.read_messages('/Combined_Covered_Indices'):
        for i in msg.data:
            if not (i in covered_array.data):
                covered_array.data.append(int(i))
    return covered_array
    
def get_latest_exploring_from_bag(b):
    t=0
    exploring_array = Int32MultiArray()
    for topic,msg,time in b.read_messages('/Combined_Exploring_Indices'):
        if time.secs > t:
            latest = msg 
            t = time.secs
        exploring_array.data=latest.data    
    return exploring_array
    
def wp_rebro(data):
    #rospy.loginfo("Received point at time %d", data.header.stamp.to_sec())
    global tare_mode
    if tare_mode:
        tare_wp = data
        pub_wp.publish(tare_wp)
    return

def vel_rebro(data):
    #rospy.loginfo("Received point at time %d", data.header.stamp.to_sec())
    global tare_mode
    if tare_mode:
        aede_vel = data
        pub_vel.publish(aede_vel)
    return

def tare_switch(tog):
    global tare_mode
    tare_mode = tog.data
    pub_wp.publish(PointStamped(header=Header(stamp=rospy.Time.now(),frame_id='map'), point=current_pose.position))
    return

def save_poi(ps):
    bag.write('/poi', ps)
    rospy.loginfo('saved poi')
    return 

def get_pos(data):
    global current_pose 
    current_pose = data.pose.pose

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('saver')
    rospy.sleep(1)
    try: 
        pub_covered.publish(get_latest_covered_from_bag(old_bag))
        pub_exploring.publish(get_latest_exploring_from_bag(old_bag))
        pub_read_vg.publish(String(folder + 'ovg.vgh'))            
        rospy.loginfo('printed')
    except:
        rospy.loginfo('L no bag')
        
    bag = rosbag.Bag(folder.joinpath(bag_name), 'w')

    rospy.Subscriber("/tare_way_point", PointStamped, wp_rebro)
    rospy.Subscriber("/aede_cmd_vel", TwistStamped, vel_rebro)
    rospy.Subscriber('/state_estimation', Odometry, get_pos)

    rospy.Subscriber('/sensor_coverage_planner/Covered_Subspace_Indices', Int32MultiArray, save_covered)
    rospy.Subscriber('/sensor_coverage_planner/Exploring_Subspace_Indices', Int32MultiArray, save_exploring)
    rospy.Subscriber('/poi', PoseStamped, save_poi) 
    rospy.Subscriber('/toggle_tare', Bool, tare_switch) 

    rospy.spin()

    bag.close()
    rospy.loginfo('bag closed')
