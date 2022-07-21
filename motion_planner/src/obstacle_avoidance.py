#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def laser_callback(scans):
    # 720/3
    ranges = [min(min(scans.ranges[0:239]),10),
              min(min(scans.ranges[240:479]),10),
              min(min(scans.ranges[480:719]),10)]
    # rospy.loginfo(ranges)
    move_state(ranges)

def move_state(range):
    msg = Twist()
    x = 0
    z = 0
    if range[0]>1 and range[1] > 1 and range[2] >1:
        x = 0.5
        z = 0
    elif range[0]>1 and range[1] < 1 and range[2] >1:
        x = 0
        z = 0.3
    elif range[0]>1 and range[1] < 1 and range[2] <1:
        x = 0
        z = 0.3
    elif range[0]<1 and range[1] < 1 and range[2] <1:
        x = 0
        z = 0.3
    elif range[0]<1 and range[1] < 1 and range[2] >1:
        x = 0
        z = 0.3
    else :
        rospy.logerr('Undefined state')
    msg.linear.x = x
    msg.angular.z = z
    pub.publish(msg)
    

def main():
    global pub
    rospy.init_node('scan_range',anonymous=True)
    sub = rospy.Subscriber("/diff_robot/laser/scan",LaserScan,laser_callback,queue_size=20)

    pub = rospy.Publisher("cmd_vel",Twist,queue_size=5)
    rospy.spin()

if __name__=="__main__":
    try:
        main()
    except:
        pass
