#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header

class SensorData(Header):
    laser_scan = LaserScan()
    odometry = Odometry()

current_header_time = 0
delta_time_threshold = 10000000
is_front_data = False
is_rear_data = False
is_odometry_data = False
total_laser_data = np.zeros(720, dtype=float)
total_odometry_data = Odometry()


def read_front_laser(data):
  global is_front_data
  global is_total_laser_data
  front_data = data.ranges
  total_laser_data[0:270] += front_data[270:540]
  total_laser_data[450:720] += front_data[0:270]
  is_front_data = True


def read_rear_laser(data):
  global is_rear_data
  global is_total_laser_data
  rear_data = data.ranges
  total_laser_data[90:630] += rear_data[0:540]
  is_rear_data = True


def read_odometry(data):
  global is_odometry_data
  global total_odometry_data
  total_odometry_data = data
  is_odometry_data = True


def reset_buffers():
  global is_front_data
  global is_rear_data
  global is_odometry_data
  global total_laser_data
  is_rear_data = False
  is_front_data = False
  is_odometry_data = False
  total_laser_data = np.zeros(720, dtype=float)


def all_data_received():
  global is_front_data
  global is_rear_data
  global total_laser_data
  global total_odometry_data
  is_rear_data = False
  is_front_data = False
  # Average the data when both lasers intersects
  total_laser_data[90:270]/=2
  total_laser_data[450:720]/=2
  total_sensor_data = SensorData()
  
  total_sensor_data.laser_scan.ranges = total_laser_data
  total_sensor_data.odometry = total_odometry_data
  
  total_sensor_data.seq = total_odometry_data.header.seq
  total_sensor_data.stamp = total_odometry_data.header.stamp
  
  pubObj.publish(total_sensor_data)


def callback_front(data):
  print("Front laser data received with header time: ", (data.header.stamp))
  global current_header_time
  global is_front_data
  global is_rear_data
  global total_laser_data
  # Correct header seq
  if abs(current_header_time - data.header.stamp.to_nsec()) < delta_time_threshold :
    read_front_laser(data)
    if is_rear_data and is_odometry_data:
      all_data_received()
      reset_buffers()
  # New header seq
  elif current_header_time < data.header.stamp.to_nsec():
    current_header_time = data.header.stamp.to_nsec()
    reset_buffers()
    read_front_laser(data)

def callback_rear(data):
  print("Rear laser data received with header time: ", (data.header.stamp))
  global current_header_time
  global is_front_data
  global is_rear_data
  global total_laser_data
  # Correct header seq
  if abs(current_header_time-data.header.stamp.to_nsec()) < delta_time_threshold :
    read_rear_laser(data)
    if is_front_data and is_odometry_data:
      all_data_received()
      reset_buffers()
  # New header seq
  elif current_header_time < data.header.stamp.to_nsec():
    current_header_time = data.header.stamp.to_nsec()
    reset_buffers()
    read_rear_laser(data)

def callback_odom(data):
  print("Odometry data received with header time: ", (data.header.stamp))
  global current_header_time
  global is_front_data
  global is_rear_data
  global total_laser_data
  # Correct header seq
  if abs(current_header_time-data.header.stamp.to_nsec()) < delta_time_threshold :
    read_odometry(data)
    if is_front_data and is_rear_data:
      all_data_received()
      reset_buffers()
  # New header seq
  elif current_header_time < data.header.stamp.to_nsec():
    current_header_time = data.header.stamp.to_nsec()
    reset_buffers()
    read_odometry(data)

if __name__ == '__main__':
  rospy.init_node("sensor_alignment")

  rate = rospy.Rate(30)
  pubObj = rospy.Publisher("/aligned_sensors", SensorData)

  subObj_odom = rospy.Subscriber("/robot/robotnik_base_control/odom", Odometry, callback_odom)
  subObj_front = rospy.Subscriber("/robot/front_laser/scan", LaserScan, callback_front)
  subObj_rear = rospy.Subscriber("/robot/rear_laser/scan", LaserScan, callback_rear)
  rospy.spin()