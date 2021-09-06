#! /usr/bin/env python

import rospy
import os
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from tf.transformations import euler_from_quaternion
from math import atan2
from points import points

SUB_TOPIC = "/odom"
PUB_TOPIC = "/cmd_vel"

# Set the initial pose of the Turtlebot3 before the first read arrived.
x = 0.0
y = 0.0
theta = 0.0



# watchdog variable
cycles=0

# newOdom extract pose from odom topic
def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (_, _, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


# Initialized the node
rospy.init_node("speed_controller")

sub = rospy.Subscriber(SUB_TOPIC, Odometry, newOdom)
pub = rospy.Publisher(PUB_TOPIC, Twist, queue_size=1)

move = Twist()
rate = rospy.Rate(5)

# Read from points until it is empty and set the goal.
goal = Point()
if points:
    new_goal = points.pop(0)
    goal.x = new_goal["x"]
    goal.y = new_goal["y"]
    rospy.loginfo("Going to [%f--%f]", goal.x, goal.y)
else:
    goal.x = 0
    goal.y = 0

# Calculate the deltas to move forward or turn right.
while not rospy.is_shutdown():

    delta_x = goal.x - x
    delta_y = goal.y - y

    angle_to_goal = atan2(delta_y, delta_x)
    delta_angle = 0.0
    if theta*angle_to_goal>0:
        delta_angle = angle_to_goal - theta
    elif theta*angle_to_goal<0:
        delta_angle = angle_to_goal + theta

    if abs(delta_angle) > 0.1:
        cycles=cycles+1
        rospy.loginfo("Turning [delta_angle=%f]", delta_angle)
        move.linear.x = 0.0
        #move.angular.z = 0.5 * delta_angle
        move.angular.z = 0.5 * delta_angle
    elif abs(delta_x) < 0.1 and abs(delta_y) < 0.1:
        cycles=0
        move.linear.x = 0.0
        move.angular.z = 0.0
        if points:
            new_goal = points.pop(0)
            goal.x = new_goal["x"]
            goal.y = new_goal["y"]
            rospy.loginfo("Next position on [%f--%f]", goal.x, goal.y)
        else:
            move.linear.x = 0.0
            move.angular.z = 0.0
            time.sleep(30)
            break
    else:
        rospy.loginfo("Going forward [delta_x=%f]", abs(delta_x))
        cycles=0
        move.linear.x = 0.3
        move.angular.z = 0.0

    if cycles>=30:
        move.linear.x = -0.3
        move.angular.z = 0.0
        cycles=0

    pub.publish(move)
    rate.sleep()

os.system("rosnode kill /speed_controller")
