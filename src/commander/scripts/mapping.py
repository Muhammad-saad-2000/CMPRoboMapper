#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from cmp_msgs.msg import SensorData


#__________________________
RESOLUTION = 0.1
OCCUPAIED_AT_END = 1
OCCUPAIED_LOG_ODD= 1.018
UNOCCUPAIED_LOG_ODD = -1.018
#__________________________
PI = 3.14159265358979323846
WIDTH=99.84/1.5
HEIGHT=99.84/1.5
X_OFFSET = 50/1.5
Y_OFFSET = 50/1.5
SENSOR_DIST=np.sqrt(2)/5+0.1
#__________________________
MAP_SIZE_HEIGHT = int(HEIGHT / RESOLUTION)
MAP_SIZE_WIDTH = int(WIDTH / RESOLUTION)

log_occupancy_grid = np.zeros((MAP_SIZE_WIDTH, MAP_SIZE_HEIGHT))

#NOTE: x, y are the actual coordinates not the indexes of the occupancy grid
#NOTE: i, j are indexes of the occupancy grid not the actual coordinates

def location_to_grid(x, y):
  return int((y + X_OFFSET)/ RESOLUTION), int((x + Y_OFFSET)/ RESOLUTION)

def grid_to_location(i, j):
  return j * RESOLUTION - X_OFFSET, i * RESOLUTION - Y_OFFSET

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
    if x0 >= 0 and x0 < MAP_SIZE_WIDTH and y0 >= 0 and y0 < MAP_SIZE_HEIGHT:
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
  # Prepare the occupancy grid message
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
  # Convert the occupancy grid to a format that can be visualized in rviz
  occupancy_grid = occupancy_grid * 120
  occupancy_grid = occupancy_grid.astype(int)
  occupancy_grid_msg.data = occupancy_grid.flatten().tolist()
  # Publish the occupancy grid
  occupancy_grid_publisher.publish(occupancy_grid_msg)

def sensor_data_callback(data):
  global log_occupancy_grid
  odometry = data.odometry
  laser_scan = data.laser_scan

  x, y= odometry.pose.pose.position.x, odometry.pose.pose.position.y # Location of the robot
  orientation = odometry.pose.pose.orientation
  theta = np.arctan2(2 * (orientation.w * orientation.z), 1 - 2 * (orientation.z * orientation.z)) # Orientation of the robot
  i_0, j_0 = location_to_grid(x, y)

  for angle, distance in zip(np.arange(len(laser_scan.ranges)) * laser_scan.angle_increment, laser_scan.ranges):
    if distance < laser_scan.range_max:
      # Systematic error correction
      if angle > PI/2 and angle < 3/2 * PI-0.005:
        j=j_0-int(SENSOR_DIST*np.sin(theta+PI/4)/RESOLUTION)
        i=i_0+int(SENSOR_DIST*np.cos(theta+PI/4)/RESOLUTION)
      else:
        j=j_0+int(SENSOR_DIST*np.sin(theta+PI/4)/RESOLUTION)
        i=i_0-int(SENSOR_DIST*np.cos(theta+PI/4)/RESOLUTION)
      points = free_grid_cells(i, j, -theta + angle + 135/180 * PI, distance)
      if len(points) > OCCUPAIED_AT_END:
        for point in points[:-OCCUPAIED_AT_END]:
          log_occupancy_grid[point] += UNOCCUPAIED_LOG_ODD
        for point in points[-OCCUPAIED_AT_END:]:
          log_occupancy_grid[point] += OCCUPAIED_LOG_ODD

  log_occupancy_grid[log_occupancy_grid > 20] = 20
  log_occupancy_grid[log_occupancy_grid < -20] = -20
  occupancy_grid = 1 - 1 / (1 + np.exp(log_occupancy_grid))
  publish_occupancy_grid(occupancy_grid)



if __name__ == '__main__':
  rospy.init_node('mapping')
  rate = rospy.Rate(30)
  occupancy_grid_publisher = rospy.Publisher('/occupancy_grid', OccupancyGrid)
  sensor_data_subscriber = rospy.Subscriber('/aligned_sensors', SensorData, sensor_data_callback)
  rospy.spin()