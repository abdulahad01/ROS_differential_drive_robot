#!/usr/bin/env python
from numpy import arctan2
import rospy 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf import transformations
import math

vel_angular = 0.3
x =0
y =0
yaw =0

def odom_callback(msg):
    global yaw,x,y
    pos = msg.pose.pose.position
    x = pos.x
    y = pos.y
    qtrn = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]
    euler = transformations.euler_from_quaternion(qtrn)
    r,p,yaw=euler



def go_to_goal(x_goal,y_goal):
    global y, x,yaw
    K_linear  = 0.1
    K_angular = 0.25
    vel = Twist()
    pub = rospy.Publisher("cmd_vel",Twist,queue_size=20)
    while 1:
        dist = math.sqrt(math.pow(y_goal-y,2)+math.pow(x_goal-x,2))
        heading = math.atan2(y_goal-y,x_goal-x)
        error = heading- yaw
        vel.angular.z= K_angular* error
        vel.linear.x = K_linear*dist
        pub.publish(vel)
        if dist<0.1:
            break
    vel.angular.z= 0
    vel.linear.x = 0
        
    


def main():
    # global pub
    rospy.init_node("go_to_goal",anonymous=True)
    sub = rospy.Subscriber("/odom",Odometry,odom_callback)
    # pub = rospy.Publisher("cmd_vel",Twist,queue_size=20)
    print("Initialized")
    go_to_goal(1,1)
    rospy.spin()


if __name__ =="__main__":
    main()
