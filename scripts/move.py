#!/usr/bin/env python

# Adds the necessary core ROS libraries for Python
import rospy

# The message used to specify velocity
from geometry_msgs.msg import Twist

# Message for laser scanner
from sensor_msgs.msg import LaserScan

global rangerobot
rangerobot = 10

def callback(data):
    global rangerobot    
    rangerobot = min(list(data.ranges))
    # get data.ranges
#    cmd_vel_x=.1
#    angle_min=data.angle_min
#    angle_increment=data.angle_increment
#    ranges = list(data.ranges)
#    range_min=data.range_min
#    angle_min=data.angle_min
#    angle_max=data.angle_max
#    angle = [(angle_min+idx*angle_increment) for idx in range(len((data.ranges)))]
# 
#    if any(data.ranges < 0.74):
#        print('STOP!')
#        cmd_vel_x=0


if __name__ == '__main__':
    rospy.init_node('move')



    # A publisher for the move data
    pub = rospy.Publisher('teleop', Twist, queue_size=1)


   #A subscriber for the scan data
    rospy.Subscriber("/scan",  LaserScan, callback)

    rospy.set_param('stop_distance',0.75)
    stop_distance=rospy.get_param('stop_distance')
    print(stop_distance)
    #Can set parameter in roscore using $ rosparam set stop_distance 1 //where 1 is the stop distance

    # Drive forward at a given speed.  The x-axis for the turlebot is "forward" relative to the direction the robot is facing. The robot turns by rotating on its z-axis
    # Ungraded exercise: what units do these velocities use?
    # Ungraded exercise: why are the other linear or angular velocities not relevant for the turtlebot?
    command = Twist()


    # Loop at 10Hz, publishing movement commands until we shut down.
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if rangerobot > stop_distance:
            command.linear.x = .1
            print(rangerobot)
        else:
            command.linear.x = 0
            print("STOP")
            print(rangerobot)
        pub.publish(command)
        rate.sleep()

