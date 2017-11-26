#! /usr/bin/env python

import yaml

import sys

import numpy as np

import roslib
import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist

GOAL_STATUSES = [
    'PENDING',
    'ACTIVE',
    'PREEMPTED',
    'SUCCEEDED',
    'ABORTED',
    'REJECTED',
    'PREEMPTING',
    'RECALLING',
    'RECALLED',
    'LOST'
]

home = MoveBaseGoal()
home.target_pose.header.frame_id = 'map'
home.target_pose.pose.position.x = 2.0
home.target_pose.pose.position.y = 2.0
home.target_pose.pose.position.z = 0.0
home.target_pose.pose.orientation.x = 0.0
home.target_pose.pose.orientation.y = 0.0
home.target_pose.pose.orientation.z = 1.0
home.target_pose.pose.orientation.w = 0.0


def interesting_behavior(publisher):
    print('Interesting Behavior?')
    turn = Twist()
    turn.angular.z = 0.25
    r = rospy.Rate(10)
    for _ in range(300):
        publisher.publish(turn)
        r.sleep()


def load_yaml(filename):
    goals = []
    with open(filename) as f:
        waypoints = yaml.load(f)
    for i, waypoint in enumerate(waypoints):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position.x = waypoint['position']['x']
        goal.target_pose.pose.position.y = waypoint['position']['y']
        goal.target_pose.pose.position.z = waypoint['position']['z']
        goal.target_pose.pose.orientation.x = waypoint['orientation']['x']
        goal.target_pose.pose.orientation.y = waypoint['orientation']['y']
        goal.target_pose.pose.orientation.z = waypoint['orientation']['z']
        goal.target_pose.pose.orientation.w = waypoint['orientation']['w']
        goals.append({'id':i, 'goal':goal})
    return goals

def get_markers(waypoints):
    markers = []
    for waypoint in waypoints:
        target = waypoint['goal'].target_pose
        marker = Marker()
        marker.header.frame_id = target.header.frame_id
        marker.id = waypoint['id']
        marker.type = 9
        marker.text = 'Marker{}'.format(waypoint['id'])
        marker.action = 0
        marker.pose = target.pose
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(0)
        markers.append(marker)
    return markers
    

if __name__ == '__main__':
    print('Patrol Node')
    rospy.init_node('patrol')
    markerPub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    twistPub = rospy.Publisher('teleop', Twist, queue_size=1)
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    waypoints = load_yaml(rospy.get_param('/explorer/patrol/waypoints'))
    markers = get_markers(waypoints)
    goal_counter = 0
    while not rospy.is_shutdown():
        goal = waypoints[goal_counter % len(waypoints)]['goal']
        goal.target_pose.header.stamp = rospy.Time.now()
        client.send_goal(goal)
        client.wait_for_result()
        interesting_behavior(twistPub)
        goal_counter += 1
