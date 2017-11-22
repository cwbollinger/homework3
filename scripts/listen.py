#!/usr/bin/env python

# Adds the necessary core ROS libraries for Python
import rospy

# The message used to specify joint state
from sensor_msgs.msg import JointState

def callback(data):
    print "Received joint data {}".format(data.name)
    print "Position: {}".format(data.position)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the one that was already running is shut down. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/joint_states", JointState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
