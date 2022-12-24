#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

#total_laser_data = np.zeros(720, dtype=float)
#front_data = np.zeros(541, dtype=float)
#rear_data = np.zeros(541, dtype=float)
current_header_seq = 0
front_data=None
rear_data=None

def callback_front(data):
  if front_data is not None and rear_data is not None:
    total_laser_data = np.zeros(720, dtype=float)
    
    # total_laser_data[0:269]=front_data
    # total_laser_data[90:630]=rear_data[0:541]
    # total_laser_data[450:719]=front_data
    
    total_laser_data[90:270]=total_laser_data[90:270]/2
    total_laser_data[550:720]=total_laser_data[550:720]/2
    
  if data.header.seq == current_header_seq:
    front_data = data.ranges
  

def callback_rear(data):
  pass

def callback_odom(data):
  pass

if __name__ == '__main__':
  rospy.init_node("sensor_alignment")

  rate = rospy.Rate(30)
  pubObj = rospy.Publisher("/aligned_sensors", Odometry)

  subObj_odom = rospy.Subscriber("/robot/robotnik_base_control/odom", Odometry, callback_odom)
  subObj_front = rospy.Subscriber("/robot/front_laser/scan", LaserScan, callback_front)
  subObj_rear = rospy.Subscriber("/robot/rear_laser/scan", LaserScan, callback_rear)
  rospy.spin()