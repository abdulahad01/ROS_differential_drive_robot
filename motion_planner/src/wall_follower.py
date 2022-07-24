#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_srvs.srv import *

ranges = {'right':0,
          'front':0,
          'left':0}
_state = 0
state_desc = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
}
pub = None
active = False

def laser_callback(scans):
    global ranges
    # 720/3
    ranges = {
              'right' : min(min(scans.ranges[0:239]),10),
              'front' : min(min(scans.ranges[240:479]),10),
              'left' : min(min(scans.ranges[480:719]),10)}
    # rospy.loginfo(ranges)
    transition()

def change_state(state):
    global _state,state_desc
    if not state == _state :
        _state = state
        print('The state changed to :',state_desc[state])

def transition():
    global ranges 
    d = 0.5
    if ranges['front'] > d and ranges['right'] > d and ranges['left'] >d:
        # no wall detected
        change_state(0)
    elif ranges['front'] < d and ranges['right'] > d and ranges['left'] >d:
        # wall in front, take a left turn
        change_state(1)
    elif ranges['front'] > d and ranges['right'] > d and ranges['left'] <d:
        # wall on the left side : keep finding the wall
        change_state(0)
    elif ranges['front'] > d and ranges['right'] < d and ranges['left'] >d:
        # wall on the right side : follow wall
        change_state(2)
    elif ranges['front'] < d and ranges['right'] > d and ranges['left'] <d:
        # front and left wall, turn left
        change_state(1)
    elif ranges['front'] < d and ranges['right'] < d and ranges['left'] <d:
        # front,right and left wall, turn left
        change_state(1)
    elif ranges['front'] < d and ranges['right'] < d and ranges['left'] >d:
        #  wall on the front and right sides, turn left
        change_state(1)
    elif ranges['front'] > d and ranges['right'] < d and ranges['left'] <d:
        #  wall on the front and right sides, turn left
        change_state(0)
    else:
        rospy.logerr('Unknown state')

def follow_wall():
    msg = Twist()
    msg.linear.x = 0.5
    msg.angular.z =0.0
    return msg

def turn_left():
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = 0.3
    return msg

def find_wall():
    msg = Twist()
    msg.linear.x = 0.5
    msg.angular.z =  0.3
    return msg

def wall_switch(req):
    global active 
    active = req.data
    resp = SetBoolResponse()
    resp.success = True
    resp.message = 'Done'
    return resp

def main():
    global pub, active
    scan_topic = "/diff_robot/laser/scan"
    vel_topic = "/cmd_vel"

    rospy.init_node('wall_follower',anonymous=True)
    pub = rospy.Publisher(vel_topic,Twist,queue_size=10)
    sub = rospy.Subscriber(scan_topic, LaserScan, laser_callback)
    srv = rospy.Service('/wall_follower_switch',SetBool,wall_switch)


    rate = rospy.Rate(10)
    print("The program is running ....")

    while not rospy.is_shutdown():
        if not active:
            rate.sleep()
            continue
        msg=Twist()
        if _state == 0:
            msg = find_wall()
        elif _state == 1:
            msg = turn_left()
        elif _state == 2:
            msg =  follow_wall()
        else:
            rospy.logerr('Unknown state')
        pub.publish(msg)
        rate.sleep()


if __name__=="__main__":
    main()
