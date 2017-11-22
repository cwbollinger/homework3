#!/usr/bin/env python

# Adds the necessary core ROS libraries for Python
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math
cvbridge = CvBridge()
from opencv_apps.msg import CircleArrayStamped, Circle, Point2D

import tf2_ros
from tf2_geometry_msgs import PointStamped

# The message used to specify velocity
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped

# Message for laser scanner
from sensor_msgs.msg import LaserScan

# Message for image
from sensor_msgs.msg import Image, LaserScan

from visualization_msgs.msg import Marker

global rangerobot
rangerobot = 10

cm_per_pixel = 1
range_scan = 5
meters_to_pixels = 100.0 * (1.0 / cm_per_pixel)
size = int(range_scan*meters_to_pixels + 50)
offset = size/2


def updatePoseEstimate(poseMsg):
    global pose_estimate
    pose_estimate = poseMsg.pose

def get_marker(x, y, marker_id):
    marker = Marker()
    marker.header.frame_id = 'map'
    marker.id = marker_id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 0.5
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.lifetime = rospy.Duration(0)
    return marker

def scancallback(data):
    angle_min=data.angle_min
    angle_increment=data.angle_increment
    ranges = list(data.ranges)
    range_min=data.range_min
    angle_min=data.angle_min
    angle_max=data.angle_max
    angle = [(angle_min+idx*angle_increment) for idx in range(len((data.ranges)))]
    length_ranges=len(ranges)
    trunc=int(length_ranges/5)
    ranges_trunc=ranges[trunc:-trunc]
    
    global rangerobot
    rangerobot = min(ranges_trunc)
    
def telecallback(data):
    global vel_data
    vel_data=data

def scanConverter(laserScan):
    global offset
    global size

    newImage = np.ones((2*size,2*size),dtype=np.uint8)*255

    for i, ray in enumerate(laserScan.ranges):
        angle = laserScan.angle_min + laserScan.angle_increment * i
        newCoordx = round(ray*meters_to_pixels*math.cos(angle))+size
        newCoordy = round(ray*meters_to_pixels*math.sin(angle))+size
        newImage[newCoordx,newCoordy] = 0
    rosImg = cvbridge.cv2_to_imgmsg(newImage,"mono8")
    imPub.publish(rosImg)

count=0

marker_list = None

def markerDetected(stamped_point):
    global pose_estimate
    global markerPub
    w = pose_estimate.pose.orientation.w
    z = pose_estimate.pose.orientation.z
    theta = math.atan2(2 * w * z, 1 - 2 * (z ** 2))
    tf = np.ones((3,3))
    tf[0,0] = math.cos(theta)
    tf[0,1] = -1 * math.sin(theta)
    tf[0,2] = pose_estimate.pose.position.x

    tf[1,0] = math.sin(theta)
    tf[1,1] = math.cos(theta)
    tf[1,2] = pose_estimate.pose.position.y

    tf[2,0] = 0
    tf[2,1] = 0
    tf[2,2] = 1

    robot_x = stamped_point.point.y
    robot_y = stamped_point.point.x

    tf2 = np.eye(3)
    tf2[0, 2] = robot_x
    tf2[1, 2] = robot_y
    full_tf = np.dot(tf, tf2)
    marker_x = full_tf[0,2]
    marker_y = full_tf[1,2]

    global marker_list
    if marker_list is None:
        marker_list = [(marker_x, marker_y)]
    else:
        found_match = False
        for i, (m_x, m_y) in enumerate(marker_list):
            dist = math.sqrt((marker_x-m_x) ** 2 + (marker_y-m_y) ** 2)
            if dist < 1.0:
                found_match = True
                marker_list[i] = ((m_x+marker_x)/2, (m_y+marker_y)/2)
                break
        if not found_match:
            marker_list.append((marker_x, marker_y))

    print('markers: {}'.format(marker_list))
    for i, marker in enumerate(marker_list):
        markerPub.publish(get_marker(marker[0], marker[1], i))


def circleProcessing(circleArray):
    global count
    if count < 30:
        count +=1
        return
    else:
        count = 0

    for circle in circleArray.circles:
        # print("Circle Center:({},{}), Radius:{}".format(circle.center.x, circle.center.y, circle.radius))
        x = (circle.center.x-size)/meters_to_pixels
        y = (circle.center.y-size)/meters_to_pixels
        marker_pos = PointStamped()
        marker_pos.header.frame_id = 'odom'
        marker_pos.header.stamp =rospy.Time()
        marker_pos.point.x=x
        marker_pos.point.y=y
        marker_pos.point.z=0.0
        # print("Adjusted Offset:({},{})".format(x, y))
        markerDetected(marker_pos)

if __name__ == '__main__':
    print('spot 1')
    global vel_data
    vel_data=None

    rospy.init_node('move')
    global tfBuffer
    tfBuffer = tf2_ros.Buffer()
    global listener
    listener = tf2_ros.TransformListener(tfBuffer)

    # Publish rviz Markers
    global markerPub
    markerPub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    # A publisher for the move data
    pub = rospy.Publisher('teleop', Twist, queue_size=1)
    
    # A pub for hough transform
    imPub = rospy.Publisher('mikecircles/image', Image, queue_size=1)

    # A subscriber for the teleop
    rospy.Subscriber('my_vel', Twist, telecallback)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, updatePoseEstimate)

    #A subscriber for the scan data
    rospy.Subscriber("/scan",  LaserScan, scanConverter)

    #A subscriber for the circle data
    circleSub = rospy.Subscriber("mikecircles/circles",  CircleArrayStamped, circleProcessing)

    rospy.set_param('stop_distance', 0.75)

    # Hough Transform Parameters
    # rospy.set_param('canny_threshold', 48.0)
    # rospy.set_param('accumulator_threshold', 46.0)
    # rospy.set_param('gaussian_blur_size', 5.0)
    # rospy.set_param('min_circle_radius', 9.0)
    # rospy.set_param('max_circle_radius', 30.0)

    stop_distance=rospy.get_param('stop_distance')
    #Can set parameter in roscore using $ rosparam set stop_distance 1 //where 1 is the stop distance

    # Drive forward at a given speed.
    command = Twist()


    # Loop at 10Hz, publishing movement commands until we shut down.
    rate = rospy.Rate(10)
    print('spot 2')
    while not rospy.is_shutdown():
        if vel_data is None:
            continue
        if rangerobot > stop_distance:
            command.linear.x = vel_data.linear.x
            command.linear.y = vel_data.linear.y
            command.linear.z = vel_data.linear.z
            command.angular.x = vel_data.angular.x
            command.angular.y = vel_data.angular.y
            command.angular.z = vel_data.angular.z

            #print(rangerobot)
        else:
            if vel_data.linear.x > 0:
                command.linear.x = 0
                #print(rangerobot)
            else:
                command.linear.x = vel_data.linear.x
                command.linear.y = vel_data.linear.y
                command.linear.z = vel_data.linear.z
                command.angular.x = vel_data.angular.x
                command.angular.y = vel_data.angular.y
                command.angular.z = vel_data.angular.z
                #print(rangerobot)
        pub.publish(command)
        rate.sleep()

