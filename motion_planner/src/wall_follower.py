#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

ranges = {'right':0,
          'front':0,
          'left':0}
_state = 0
pub = None

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
    global _state
    if not _state == state :
        _state = state
        print('The state changed to :',state)

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

def main():
    global pub 
    scan_topic = "/diff_robot/laser/scan"
    vel_topic = "/cmd_vel"
    rospy.init_node('wall_follower',anonymous=True)
    pub = rospy.Publisher(vel_topic,Twist,queue_size=10)
    sub = rospy.Subscriber(scan_topic, LaserScan, laser_callback)
    rate = rospy.Rate(10)
    print("The program is running ....")

    while not rospy.is_shutdown():
        msg=Twist()
        if _state == 0:
            msg = find_wall()
        elif _state == 1:
            msg = turn_left()
        elif _state == 2:
            msg =  follow_wall()
        elif _state == 3:
            msg =  turn_right()
        else:
            rospy.logerr('Unknown state')
        pub.publish(msg)
        rate.sleep()


if __name__=="__main__":
    main()