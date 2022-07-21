#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf import transformations
import math
from std_srvs.srv import SetBool

ranges = {'right':0,
          'front':0,
          'left':0}
_state = 0
pub = None
wall_follow_client_srv = None
go_to_goal_client_srv = None

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



def laser_callback(scans):
    global ranges
    # 720/3
    ranges = {
              'right' : min(min(scans.ranges[0:239]),10),
              'front' : min(min(scans.ranges[240:479]),10),
              'left' : min(min(scans.ranges[480:719]),10)}
    # rospy.loginfo(ranges)

def change_state(state):
    global _state
    _state = state
    if state ==  0:
        resp = go_to_goal_client_srv(True)
        resp = wall_follow_client_srv(False)
    else :
        resp = go_to_goal_client_srv(False)
        resp = wall_follow_client_srv(True)



def main():

    scan_topic = "/diff_robot/laser/scan"
    vel_topic = "/cmd_vel"
    odom_topic = "/odom"

    rospy.init_node('bug_0',anonymous=True)

    pub = rospy.Publisher(vel_topic,Twist,queue_size=10)
    sub_odoms = rospy.Subscriber(odom_topic,Odometry,odom_callback)
    sub_scans = rospy.Subscriber(scan_topic, LaserScan, laser_callback)
    rate = rospy.Rate(10)

    msg = Twist()

    print("The program is running ....")
    while not rospy.is_shutdown():
        if  _state == None:
            continue
        
        if  _state == 0:
            if ranges['front'] > 0.15 and ranges['front'] < 1:
                change_state(1)
        
        elif _state == 1:
            desired_yaw = math.atan2(desired_position_.y - position_.y, desired_position_.x - position_.x)
            err_yaw = normalize_angle(desired_yaw - yaw_)
            
            # less than 30 degrees
            if math.fabs(err_yaw) < (math.pi / 6) and \
             ranges['front'] > 1.5 and ranges['fright'] > 1 and ranges['fleft'] > 1:
                change_state(0)
            
            # between 30 and 90
            if err_yaw > 0 and \
               math.fabs(err_yaw) > (math.pi / 6) and \
               math.fabs(err_yaw) < (math.pi / 2) and \
             ranges['left'] > 1.5 and ranges['fleft'] > 1:
                change_state(0)
                
            if err_yaw < 0 and \
               math.fabs(err_yaw) > (math.pi / 6) and \
               math.fabs(err_yaw) < (math.pi / 2) and \
             ranges['right'] > 1.5 and ranges['fright'] > 1:
                change_state(0)            
        rate.sleep()


if __name__ =="__main__":
    try:
        main()
    except:
        pass

