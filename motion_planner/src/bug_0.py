#!/usr/bin/env python

import rospy

# import mesaage formats
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf import transformations

import math

# import service message of type setBool
from std_srvs.srv import SetBool

# Initialize necessary global variables
ranges = {'right':0,
          'front':0,
          'left':0}
_state = 0
x =0
y =0
yaw =0

pub = None

wall_follow_client_srv = None
go_to_goal_client_srv = None

# get goal position from goal param
goal_position = Point()
goal_position.x = rospy.get_param('goal_x')
goal_position.y = rospy.get_param('goal_y')
goal_position.z = 0

# call back function for odom subscriber
def odom_callback(msg):
    global yaw,x,y
    pos = msg.pose.pose.position
    x = pos.x
    y = pos.y
    qtrn = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]
    euler = transformations.euler_from_quaternion(qtrn)
    r,p,yaw=euler

# call back function for laser scans
def laser_callback(scans):
    global ranges
    # 720/3
    ranges = {
              'right' : min(min(scans.ranges[0:239]),10),
              'front' : min(min(scans.ranges[240:479]),10),
              'left' : min(min(scans.ranges[480:719]),10)}

#  state machine function
def change_state(state):
    global _state
    _state = state
    # 0 : go to goal
    # 1 : wall follow
    if state ==  0:
        resp = go_to_goal_client_srv(True)
        resp = wall_follow_client_srv(False)
    else :
        resp = go_to_goal_client_srv(False)
        resp = wall_follow_client_srv(True)

# main function all the behavioral logic goes here
def main():
    global y, x,yaw,ranges,goal_position, _state
    global wall_follow_client_srv,go_to_goal_client_srv

    scan_topic = "/diff_robot/laser/scan"
    odom_topic = "/odom"

    rospy.init_node('bug_0',anonymous=True)

    sub_odoms = rospy.Subscriber(odom_topic,Odometry,odom_callback)
    sub_scans = rospy.Subscriber(scan_topic, LaserScan, laser_callback)

    rospy.wait_for_service('/go_to_goal_switch')
    rospy.wait_for_service('/wall_follower_switch')

    wall_follow_client_srv = rospy.ServiceProxy('/wall_follower_switch',SetBool)
    go_to_goal_client_srv = rospy.ServiceProxy('/go_to_goal_switch',SetBool)

    rate = rospy.Rate(10)

    change_state(0)

    print("The program is running ....")
    while not rospy.is_shutdown():        
        if  _state == 0:
            if ranges['front'] > 0.15 and ranges['front'] < 1:
                change_state(1)
                rospy.loginfo("FOllowing wall")
        
        elif _state == 1:
            heading = math.atan2(goal_position.y-y,goal_position.x-x)
            err_yaw = heading- yaw
            print
            
            # less than 30 degrees
            if math.fabs(err_yaw) < (math.pi / 6) and \
             ranges['front'] > 1.5 and ranges['right'] > 1 and ranges['left'] > 1:
                change_state(0)
                rospy.loginfo("going to goal")
            
            # between 30 and 90
            if err_yaw > 0 and \
               math.fabs(err_yaw) > (math.pi / 6) and \
               math.fabs(err_yaw) < (math.pi / 2) and \
             ranges['left'] > 1.5 and ranges['left'] > 1:
                change_state(0)
                rospy.loginfo("going to goal")

                
            if err_yaw < 0 and \
               math.fabs(err_yaw) > (math.pi / 6) and \
               math.fabs(err_yaw) < (math.pi / 2) and \
             ranges['right'] > 1.5 and ranges['right'] > 1:
                change_state(0)            
                rospy.loginfo("going to goal")

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except:
        pass

