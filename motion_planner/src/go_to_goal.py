#!/usr/bin/env python

import rospy 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from tf import transformations
from std_srvs.srv import *
import math

vel_angular = 0.3
x =0
y =0
yaw =0
active = False
pub = None

position = Point()

desired_position = Point()
desired_position.x = rospy.get_param('goal_x')
desired_position.y = rospy.get_param('goal_y')
desired_position.z = 0

def gtg_switch(req):
    global active
    active = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done'
    return res

def odom_callback(msg):
    global yaw,position
    position = msg.pose.pose.position

    qtrn = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]
    euler = transformations.euler_from_quaternion(qtrn)
    r,p,yaw=euler

def go_to_goal(point):
    global position,yaw
    x = position.x
    y = position.y
    x_goal = point.x
    y_goal = point.y
    K_linear  = 0.1
    K_angular = 0.25
    vel = Twist()
    while 1:
        dist = math.sqrt(math.pow(y_goal-y,2)+math.pow(x_goal-x,2))
        heading = math.atan2(y_goal-y,x_goal-x)
        error = heading- yaw
        vel.angular.z= K_angular* error
        vel.linear.x = K_linear*dist
        if dist<0.1:
            break
        return vel
    vel.angular.z= 0
    vel.linear.x = 0
    return vel
        

def main():
    global pub,desired_position,active

    rospy.init_node("go_to_goal",anonymous=True)

    sub = rospy.Subscriber("/odom",Odometry,odom_callback)
    pub = rospy.Publisher('/cmd_vel',Twist,queue_size =10)
    srv = rospy.Service("/go_to_goal_switch",SetBool,gtg_switch)

    rate = rospy.Rate(10)

    print("Running go to goal")
    while not rospy.is_shutdown():
        if not active:
            continue
        else:
            msg = go_to_goal(desired_position)
            pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    main()
