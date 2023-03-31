#!/usr/bin/env python3
# license removed for brevity
import rospy
# from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from sensor_msgs.msg import Imu
import pyzed.sl as sl
import cv2
import numpy as np
import math
import scipy

# # 
# Basic class to handle the timestamp of the different sensors to know if it is a new sensors_data or an old one
class TimestampHandler:
    def __init__(self):
        self.t_imu = sl.Timestamp()

    # check if the new timestamp is higher than the reference one, and if yes, save the current as reference
    def is_new(self, sensor):
        if (isinstance(sensor, sl.IMUData)):
            new_ = (sensor.timestamp.get_microseconds() > self.t_imu.get_microseconds())
            if new_:
                self.t_imu = sensor.timestamp
            return new_

def talker(imu_data):
    pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    # rospy.loginfo(imu_data)
    pub.publish(imu_data)
    rate.sleep()

def main():
    # Create a Camera object
    zed = sl.Camera()

    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.NONE

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS :
        print(repr(err))
        zed.close()
        exit(1)

    # Get camera information sensors_data
    info = zed.get_camera_information()

    cam_model = info.camera_model
    if cam_model == sl.MODEL.ZED :
        print("This tutorial only supports ZEDM and ZED2 camera models, ZED does not have additional sensors")
        exit(1)
    
    # Used to store the sensors timestamp to know if the sensors_data is a new one or not
    ts_handler = TimestampHandler()

    # Get Sensor Data for 5 seconds
    # i = 0
    sensors_data = sl.SensorsData()

    while not rospy.is_shutdown():
        # retrieve the current sensors sensors_data
        # Depending on your Camera model or its firmware, differents sensors are presents.
        # They do not run at the same rate: Therefore, to do not miss samples we iterate as fast as we can and compare timestamp to know when a sensors_data is a new one
        # NOTE: There is no need to acquire images with grab() function. Sensors sensors_data are running in a separated internal capture thread.
        if zed.get_sensors_data(sensors_data, sl.TIME_REFERENCE.CURRENT) == sl.ERROR_CODE.SUCCESS :
            # Check if the data has been updated since the last time
            # IMU is the sensor with the highest rate
            if ts_handler.is_new(sensors_data.get_imu_data()):

                imu_data_zed = sensors_data.get_imu_data()

                # Filtered orientation quaternion
                quaternion = sensors_data.get_imu_data().get_pose().get_orientation().get()
                # print(" \t Orientation: [ Ox: {0}, Oy: {1}, Oz {2}, Ow: {3} ]".format(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
                
                # linear acceleration
                linear_acceleration = sensors_data.get_imu_data().get_linear_acceleration()
                # print(" \t Acceleration: [ {0} {1} {2} ] [m/sec^2]".format(linear_acceleration[0], linear_acceleration[1], linear_acceleration[2]))

                # angular velocities
                angular_velocity = sensors_data.get_imu_data().get_angular_velocity()
                # print(" \t Angular Velocities: [ {0} {1} {2} ] [deg/sec]".format(angular_velocity[0], angular_velocity[1], angular_velocity[2]))

                # i = i+1

                imu_data_final = Imu()

                rospy.init_node('talker_imu', anonymous=True)

                imu_data_final.header.stamp = rospy.Time.now()

                imu_data_final.orientation.x = quaternion[0]
                imu_data_final.orientation.y = quaternion[1]
                imu_data_final.orientation.z = quaternion[2]
                imu_data_final.orientation.w = quaternion[3]

                imu_data_final.linear_acceleration.x = linear_acceleration[0]
                imu_data_final.linear_acceleration.y = linear_acceleration[1]                
                imu_data_final.linear_acceleration.z = linear_acceleration[2]

                imu_data_final.angular_velocity.x = angular_velocity[0]
                imu_data_final.angular_velocity.y = angular_velocity[1]
                imu_data_final.angular_velocity.z = angular_velocity[2]

                talker(imu_data_final)

    zed.close()
    return 0

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
