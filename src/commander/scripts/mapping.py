#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from noise import pnoise2
from nav_msgs.msg import OccupancyGrid

class SensorData(Header):
  laser_scan = LaserScan()
  odometry = Odometry()

PI = 3.14159265358979323846
X_OFFSET = 50
Y_OFFSET = 50
RESOLUTION = 0.02 # 0.02
MAP_SIZE_WIDTH = 4992
MAP_SIZE_HEIGHT = 4992

MAX_SUPPORTED=1000# for the belief of map cell occupancy

#NOTE: x,y are the actual coordinates not the indexes of the occupancy grid
def location_to_grid(x, y):
  return int((x + X_OFFSET)/ RESOLUTION), int((y + Y_OFFSET)/ RESOLUTION)

#NOTE: i,j are indexes of the occupancy grid not the actual coordinates
def grid_to_location(i, j):
  return i * RESOLUTION - X_OFFSET, j * RESOLUTION - Y_OFFSET

#NOTE: i,j are indexes of the occupancy grid not the actual coordinates
def free_grid_cells(i, j, angle, distance):
  points = []
  x0 = i
  y0 = j
  x1 = i + int(distance / RESOLUTION * np.cos(angle))
  y1 = j + int(distance / RESOLUTION * np.sin(angle))
  dx = abs(x1 - x0)
  dy = abs(y1 - y0)
  sx = 1 if x0 < x1 else -1
  sy = 1 if y0 < y1 else -1
  err = dx - dy
  while True:
    points.append((x0, y0))
    if x0 == x1 and y0 == y1:
      break
    e2 = 2 * err
    if e2 > -dy:
      err = err - dy
      x0 = x0 + sx
    if e2 < dx:
      err = err + dx
      y0 = y0 + sy
  return points

def publish_occupancy_grid(occupancy_grid):
  occupancy_grid_msg = OccupancyGrid()
  occupancy_grid_msg.header.stamp = rospy.Time.now()
  occupancy_grid_msg.header.frame_id = "robot_map"
  occupancy_grid_msg.info.resolution = RESOLUTION
  occupancy_grid_msg.info.width = MAP_SIZE_WIDTH
  occupancy_grid_msg.info.height = MAP_SIZE_HEIGHT
  occupancy_grid_msg.info.origin.position.x = -X_OFFSET
  occupancy_grid_msg.info.origin.position.y = -Y_OFFSET
  occupancy_grid_msg.info.origin.position.z = 0
  occupancy_grid_msg.info.origin.orientation.x = 0
  occupancy_grid_msg.info.origin.orientation.y = 0
  occupancy_grid_msg.info.origin.orientation.z = 0
  occupancy_grid_msg.info.origin.orientation.w = 1
  occupancy_grid_msg.data = occupancy_grid.flatten().tolist()
  occupancy_grid_publisher.publish(occupancy_grid_msg)

def sensor_data_callback(sensor_data):
  odometry = sensor_data.odometry
  laser_scan = sensor_data.laser_scan

  x, y, theta = odometry.pose.pose.position.x, odometry.pose.pose.position.y, odometry.pose.pose.orientation.z
  i, j = location_to_grid(x, y)

  for angle, distance in zip(np.arange(len(laser_scan.ranges)) * laser_scan.angle_increment, laser_scan.ranges):
    if distance < laser_scan.range_max:
      for i_itr, j_itr in free_grid_cells(i, j, theta + angle, distance):
        if i_itr >= 0 and i_itr < MAP_SIZE_WIDTH and j_itr >= 0 and j_itr < MAP_SIZE_HEIGHT:
          occupancy_grid[i_itr, j_itr] += (1 - occupancy_grid[i_itr, j_itr]) * 0.1
  
  publish_occupancy_grid(occupancy_grid)



if __name__ == '__main__':
  rospy.init_node('mapping')
  rospy.loginfo('mapping node started')
  occupancy_grid = np.zeros((MAP_SIZE_WIDTH, MAP_SIZE_HEIGHT))
  
  occupancy_grid_publisher = rospy.Publisher('/occupancy_grid', OccupancyGrid)
  rate = rospy.Rate(10)
  
  sensor_data = SensorData()
  sensor_data_subscriber = rospy.Subscriber('/aligned_sensors', SensorData, sensor_data_callback)
  rospy.spin()
