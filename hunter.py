#!/usr/bin/env python

import rospy 
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import PointStamped, PoseStamped, Point
from nav_msgs.msg import Odometry
from yolov7_ros.msg import Detection, Detections
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo, PointCloud2
from sensor_msgs import point_cloud2
import tf
from tf.transformations import quaternion_matrix, translation_matrix
import message_filters
import numpy as np
from visualization_msgs.msg import Marker

pub_poi = rospy.Publisher('/poi_out', PointStamped, queue_size=10)
pub_vis = rospy.Publisher('/poi_map', Marker, queue_size=10)

cam = PinholeCameraModel()
poi_list = []

def poi_callback(det_msg, scan_msg, info_msg):
    global poi_list
    cam.fromCameraInfo(info_msg)
    for det in det_msg.dets:
        center = ((det.bbox[0]+det.bbox[2])/2, (det.bbox[1]+det.bbox[3])/2)
        center_rectified = cam.rectifyPoint(center)
        vector = cam.projectPixelTo3dRay(center_rectified)
        try:
            (trans,rot) = listener.lookupTransform('/map', '/camera', rospy.Time.now())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return
        rot_mat = quaternion_matrix(rot)
        trans_mat = translation_matrix(trans)
        vector1 = [vector[0], vector[1], vector[2], 1]
        vector_array = np.array(vector1)
        vector_rot = np.matmul(rot_mat, vector_array)
        vector_tf = vector_rot.tolist()
        vector_tf.pop(3)
        cam_array = np.array([0,0,0,1])
        cam_trans = np.matmul(trans_mat, cam_array)
        cam_origin = cam_trans.tolist()
        cam_origin.pop(3)

        possible_points = {}
        poins = point_cloud2.read_points(scan_msg, skip_nans=True, field_names=("x", "y", "z"))
        for pp in poins:
            kx = (pp[0]-cam_origin[0])/vector_tf[0]
            ky = (pp[1]-cam_origin[1])/vector_tf[1]
            kz = (pp[2]-cam_origin[2])/vector_tf[2]
            ks = [kx,ky,kz]
            kd = np.std(ks)
            p3 = np.array(pp)
            p1 = np.array(cam_origin)
            p2 = p1+np.array(vector_tf)
            #find dist of p3 from line p1p2
            dist=np.cross(p2-p1,p3-p1)/np.linalg.norm(p2-p1)
            if dist[0]<0.1 and kx>0 and ky>0 and kz>0:
                possible_points[kd] = pp
        key = min(possible_points.keys())
        poi_xyz = possible_points[key]
        poi = PointStamped(header=Header(stamp=rospy.Time.now(),frame_id=det.name), point=Point(poi_xyz[0], poi_xyz[1], poi_xyz[2]))
        print(poi)
        if compare_pois(poi, det.name, 5):
            poi_list.append(poi)
            rospy.loginfo(len(poi_list))
            rospy.loginfo(poi)
            pub_poi.publish(poi)

    generate_visual(poi_list)

def compare_pois(poi, name, r):
    for prev_poi in poi_list:
        if prev_poi.header.frame_id != name:
            continue
        dx = prev_poi.point.x - poi.point.x
        dy = prev_poi.point.y - poi.point.y
        dxy = dx**2 + dy**2
        if dxy < r**2:
            return False
    return True

def generate_visual(list):
    marker = Marker()
    marker.header = Header(frame_id = 'map', stamp = rospy.Time.now())
    marker.id = 259683
    marker.type = 7
    marker.scale.x = 1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.pose.orientation.w = 1
    for point in list:
        marker.points.append(point.point)
        if point.header.frame_id == 'door':
            marker.colors.append(ColorRGBA(1.0,0.9,0.1,1.0))
        elif point.header.frame_id == 'person':
            marker.colors.append(ColorRGBA(1.0,0.1,0.1,1.0))
        else:
            marker.colors.append(ColorRGBA(1.0,1.0,1.0,1.0))
    pub_vis.publish(marker)

if __name__ == '__main__':
    rospy.init_node('hunter')
    listener = tf.TransformListener()
    det_sub = message_filters.Subscriber('/detections', Detections)
    scan_sub = message_filters.Subscriber('/registered_scan', PointCloud2)
    info_sub = message_filters.Subscriber('/camera/camera_info', CameraInfo)
    ts = message_filters.ApproximateTimeSynchronizer([det_sub, scan_sub, info_sub], 10, 0.1)
    ts.registerCallback(poi_callback)
    rospy.spin()
