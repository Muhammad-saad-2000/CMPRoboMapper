#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header

class SensorData(Header):
  laser_scan = LaserScan()
  odometry = Odometry()

current_time_sequance = 0
DELTA_TIME_MAX = 10000000
front_data_in_sequance = False
rear_data_in_sequance = False
odometry_data_in_sequance = False
incorporated_laser_data = np.zeros(720, dtype=float)
odometry_data = Odometry()


def read_front_laser(data):
  global front_data_in_sequance
  global is_total_laser_data
  front_data = data.ranges
  incorporated_laser_data[0:270] += front_data[270:540]
  incorporated_laser_data[450:720] += front_data[0:270]
  front_data_in_sequance = True


def read_rear_laser(data):
  global rear_data_in_sequance
  global is_total_laser_data
  rear_data = data.ranges
  incorporated_laser_data[90:630] += rear_data[0:540]
  rear_data_in_sequance = True


def read_odometry(data):
  global odometry_data_in_sequance
  global odometry_data
  odometry_data = data
  odometry_data_in_sequance = True


def reset_buffers():
  global front_data_in_sequance
  global rear_data_in_sequance
  global odometry_data_in_sequance
  global incorporated_laser_data
  rear_data_in_sequance = False
  front_data_in_sequance = False
  odometry_data_in_sequance = False
  incorporated_laser_data = np.zeros(720, dtype=float)


def all_data_received():
  global front_data_in_sequance
  global rear_data_in_sequance
  global incorporated_laser_data
  global odometry_data
  rear_data_in_sequance = False
  front_data_in_sequance = False
  # Average the data when both lasers intersects
  incorporated_laser_data[90:270]/=2
  incorporated_laser_data[450:720]/=2
  total_sensor_data = SensorData()
  
  total_sensor_data.laser_scan.ranges = incorporated_laser_data
  total_sensor_data.laser_scan.angle_min = 0
  total_sensor_data.laser_scan.angle_max = 6.274446
  total_sensor_data.odometry = odometry_data
  
  total_sensor_data.seq = odometry_data.header.seq
  total_sensor_data.stamp = odometry_data.header.stamp
  
  pubObj.publish(total_sensor_data)


def callback_front(data):
  global current_time_sequance
  global front_data_in_sequance
  global rear_data_in_sequance
  global incorporated_laser_data
  # Correct time slot
  if abs(current_time_sequance - data.header.stamp.to_nsec()) < DELTA_TIME_MAX :
    read_front_laser(data)
    if rear_data_in_sequance and odometry_data_in_sequance:
      all_data_received()
      reset_buffers()
  # New time slot
  elif current_time_sequance < data.header.stamp.to_nsec():
    current_time_sequance = data.header.stamp.to_nsec()
    reset_buffers()
    read_front_laser(data)

def callback_rear(data):
  global current_time_sequance
  global front_data_in_sequance
  global rear_data_in_sequance
  global incorporated_laser_data
  # Correct time slot
  if abs(current_time_sequance-data.header.stamp.to_nsec()) < DELTA_TIME_MAX :
    read_rear_laser(data)
    if front_data_in_sequance and odometry_data_in_sequance:
      all_data_received()
      reset_buffers()
  # New time slot
  elif current_time_sequance < data.header.stamp.to_nsec():
    current_time_sequance = data.header.stamp.to_nsec()
    reset_buffers()
    read_rear_laser(data)

def callback_odom(data):
  global current_time_sequance
  global front_data_in_sequance
  global rear_data_in_sequance
  global incorporated_laser_data
  # Correct time slot
  if abs(current_time_sequance-data.header.stamp.to_nsec()) < DELTA_TIME_MAX :
    read_odometry(data)
    if front_data_in_sequance and rear_data_in_sequance:
      all_data_received()
      reset_buffers()
  # New time slot
  elif current_time_sequance < data.header.stamp.to_nsec():
    current_time_sequance = data.header.stamp.to_nsec()
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