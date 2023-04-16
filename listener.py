#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + '\n%s \n %s', data.linear_acceleration, data.angular_velocity)

def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('imu/data', Imu, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
