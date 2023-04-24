import rospy
import rosbag
from std_msgs.msg import Int32MultiArray, String, Bool, Header
from geometry_msgs.msg import PointStamped, Pose, PoseStamped
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import DeleteModel
from pathlib import Path
import cv2

pub_wp = rospy.Publisher('/way_point', PointStamped, queue_size=1)
pub_poi = rospy.Publisher('/poi_out', PoseStamped, queue_size=10)
pub_kill = rospy.Publisher('/del_model', String, queue_size=5)

tare_mode = True
current_pose = Pose()
door_list = [['door1', 43, -9], ['door2', 85, -4], ['door4', 56, 57], ['door3', 10, -2]]

script_dir = Path( __file__ ).parent.absolute()
start_path = str(script_dir.joinpath('start.png'))
start = cv2.imread(start_path)

def del_model(model_name : str):
    rospy.wait_for_service('/gazebo/delete_model')
    del_model_proxy = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    del_model_proxy(model_name)
    rospy.loginfo('destroyed' + model_name)
    return

def del_model_sel(model):
    d = 2**8
    m = model.data
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
    del_model(m)


def wp_rebro(data):
    #rospy.loginfo("Received point at time %d", data.header.stamp.to_sec())
    global tare_mode
    if tare_mode:
        tare_wp = data
        pub_wp.publish(tare_wp)
    return

def tare_switch(tog):
    global tare_mode
    tare_mode = tog.data
    pub_wp.publish(PointStamped(header=Header(stamp=rospy.Time.now(),frame_id='map'), point=current_pose.position))
    return

def get_pos(data):
    global current_pose 
    current_pose = data.pose.pose

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('saver')
    while(True):
        cv2.imshow('waiting...', start)
        k = cv2.waitKey(0)
        print(k)
        if k == 27:
            cv2.destroyWindow('waiting...')
            break
    pub_poi.publish(PoseStamped(header=Header(stamp=rospy.Time.now(),frame_id='test'), pose=current_pose))
    pub_kill.publish(String('test'))

    rospy.Subscriber("/tare_way_point", PointStamped, wp_rebro)
    rospy.Subscriber('/state_estimation', Odometry, get_pos)
    rospy.Subscriber('/toggle_tare', Bool, tare_switch) 
    rospy.Subscriber('/del_model', String, del_model_sel)
    rospy.spin()