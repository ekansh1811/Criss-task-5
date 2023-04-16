#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
import numpy as np
import time

# Define the frequency of the IMU data
freq = 100

# Define the initial values of the accelerometer and gyroscope
accel_init = np.array([0, 0, 9.81])
gyro_init = np.array([0, 0, 0])

# Define the noise level for the accelerometer and gyroscope
accel_noise = 0.05
gyro_noise = 0.05

# Define the correction matrices for the accelerometer and gyroscope
accel_correction = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
gyro_correction = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

# Define the ROS node and publisher
rospy.init_node('imu_generator')
pub = rospy.Publisher('imu/data', Imu, queue_size=10)

# Define the loop that generates and publishes the IMU data
rate = rospy.Rate(freq)
start_time = time.time()
init_time = 0
k = 0
while not rospy.is_shutdown():
    if k>=10:
        quit()
    init_time+= time.time()-start_time
    k = init_time/1000
    # Generate the accelerometer and gyroscope values
    accel = accel_init + np.array([k,k,0])
    gyro = gyro_init + np.random.normal(0, gyro_noise, 3)

    # Apply the correction matrices to the accelerometer and gyroscope
    accel_corr = np.dot(accel_correction, accel)
    gyro_corr = np.dot(gyro_correction, gyro)

    # Create the IMU message and populate it with the data
    imu_msg = Imu()
    imu_msg.header.stamp = rospy.Time.now()
    imu_msg.header.frame_id = 'imu_link'
    imu_msg.linear_acceleration.x = accel_corr[0]
    imu_msg.linear_acceleration.y = accel_corr[1]
    imu_msg.linear_acceleration.z = accel_corr[2]
    imu_msg.angular_velocity.x = gyro_corr[0]
    imu_msg.angular_velocity.y = gyro_corr[1]
    imu_msg.angular_velocity.z = gyro_corr[2]
    

    # Publish the IMU message
    pub.publish(imu_msg)

    # Sleep to maintain the frequency
    rate.sleep()
