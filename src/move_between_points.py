#! /usr/bin/env python

import rospy
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
    delta_angle = angle_to_goal - theta

    if abs(delta_angle) > 0.1:
        rospy.loginfo("Turning right [delta_angle=%f]", delta_angle)
        move.linear.x = 0.0
        move.angular.z = 0.3
    elif abs(delta_x) < 0.1:
        move.linear.x = 0.0
        move.angular.z = 0.0
        if points:
            new_goal = points.pop(0)
            goal.x = new_goal["x"]
            goal.y = new_goal["y"]
            rospy.loginfo("Next position on [%f--%f]", goal.x, goal.y)
        else:
            break
    else:
        rospy.loginfo("Going forward [delta_x=%f]", abs(delta_x))
        move.linear.x = 0.3
        move.angular.z = 0.0

    pub.publish(move)
    rate.sleep()