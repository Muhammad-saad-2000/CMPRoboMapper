#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from cmp_msgs.msg import SensorData
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
import os
import matplotlib.pyplot as plt
PI = 3.14159265358979323846
SENSOR_DIST=np.sqrt(2)/5+0.1
PARTICLES_NUM = 7 # Number of particles
# Map Parameters
#__________________________
RESOLUTION = 0.1
#__________________________
PI = 3.14159265358979323846
WIDTH=99.84
HEIGHT=99.84
X_OFFSET = 50
Y_OFFSET = 50
SENSOR_DIST=np.sqrt(2)/5+0.1
#__________________________
MAP_SIZE_HEIGHT = int(HEIGHT / RESOLUTION)
MAP_SIZE_WIDTH = int(WIDTH / RESOLUTION)
NUM_RAYS = 720
ANGLE_INCREMENT = 0.0087266
MAX_RANGE = 30
OCCUPAIED_AT_END = 2
OCCUPAIED_LOG_ODD= 1.118
UNOCCUPAIED_LOG_ODD = -1.018
# Initialize the particles
X_prev = np.hstack((np.random.uniform(0 - 0.005, 0 + 0.005, size=(PARTICLES_NUM, 1)),
                    np.random.uniform(0 - 0.005, 0 +0.005, size=(PARTICLES_NUM, 1)),
                     np.random.uniform(-0.05 , 0.05, size=(PARTICLES_NUM, 1))
                    ))


def location_to_grid(x, y):
  i=int((y + X_OFFSET)/ RESOLUTION)
  j=int((x + Y_OFFSET)/ RESOLUTION)
  if i>=MAP_SIZE_WIDTH:
    i=MAP_SIZE_WIDTH-1
  if j>=MAP_SIZE_HEIGHT:
    j=MAP_SIZE_HEIGHT-1
  return i, j

grid_to_location = lambda i, j: (j * RESOLUTION - X_OFFSET, i * RESOLUTION - Y_OFFSET)

pose_prev_truth = [0, 0, 0]
current_pos_estimite= None
current_map_estimite=np.zeros((MAP_SIZE_HEIGHT,MAP_SIZE_WIDTH))
M=np.zeros((PARTICLES_NUM,MAP_SIZE_HEIGHT,MAP_SIZE_WIDTH))


def publish_log_odds_occupancy_grid(m):
  occupancy_grid = np.round(1 / (1 + np.exp(-m))) # Binary map (occupancy grid)
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
  map_publisher.publish(occupancy_grid_msg)


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


def update_map(x, z, m):
  log_occupancy_grid = m
  laser_scan = z
  xₚ = x[0]
  yₚ = x[1]
  θₚ = x[2]
  i_0, j_0 = location_to_grid(xₚ, yₚ)
  for angle, distance in zip(np.arange(len(laser_scan.ranges)) * laser_scan.angle_increment, laser_scan.ranges):
    if distance < laser_scan.range_max:
      # Systematic error correction
      if angle > PI/2-0.004 and angle < 3/2 * PI-0.005:
        j=j_0-int(SENSOR_DIST*np.sin(θₚ+PI/4)/RESOLUTION)
        i=i_0+int(SENSOR_DIST*np.cos(θₚ+PI/4)/RESOLUTION)
      else:
        j=j_0+int(SENSOR_DIST*np.sin(θₚ+PI/4)/RESOLUTION)
        i=i_0-int(SENSOR_DIST*np.cos(θₚ+PI/4)/RESOLUTION)
      points = free_grid_cells(i, j, -θₚ + angle + 135/180 * PI, distance)
      if len(points) > OCCUPAIED_AT_END:
        for point in points[:-OCCUPAIED_AT_END]:
          log_occupancy_grid[point] += UNOCCUPAIED_LOG_ODD
        for point in points[-OCCUPAIED_AT_END:]:
          log_occupancy_grid[point] += OCCUPAIED_LOG_ODD

  log_occupancy_grid[log_occupancy_grid > 20] = 20
  log_occupancy_grid[log_occupancy_grid < -20] = -20
  return log_occupancy_grid


def p_sensor_model(x, z, m):
  occupancy_grid = np.round(1 / (1 + np.exp(-m))) # Binary map (occupancy grid)
  occupancy_grid = occupancy_grid.astype(np.uint8) * 255
  # Transform the map into a likelihood field
  ll_field = cv2.distanceTransform(occupancy_grid, cv2.DIST_L2, 0)
  σ, π = 13, 3.14
  ll_field = np.array(ll_field)
  ll_field = 1/np.sqrt(2*π*σ) * np.exp(-0.5*((ll_field)/σ)**2)
  
  # Modify the likelihood field to account for random noise
  norm = np.max(ll_field)
  ll_field = (0.991 * ll_field)/norm
  # To be added in case of max range reading
  max_range_weight = 0.009 * 1/((30-27) * norm)
  
  xt, yt, θt = x
  
  xₑ, yₑ = project(z, xt, yt, θt)
  p = 0
  for i, (xb, yb) in enumerate(zip(xₑ, yₑ)):
      if xb == -np.inf or yb == -np.inf:# The ray has crossed the boundary of the map
        p += np.log(10e-3)/100
      else:
        ib, jb = location_to_grid(xb, yb)
        p += np.log(ll_field[ib, jb] + (max_range_weight if MAX_RANGE-0.5 < z.ranges[i] <= MAX_RANGE else 0))/100
  return p


def sample_motion_model(estimated_pose_prev, u_t):
    # Previous state 
    X, Y, θ = estimated_pose_prev
    # Action
    pose_u_prev, pose_u = u_t
    X_u, Y_u, θ_u = pose_u
    X_uprev, Y_uprev, θ_uprev = pose_u_prev
    
    # Difference due to odometry
    Δδtr_u = np.sqrt((X_u - X_uprev)**2 + (Y_u - Y_uprev)**2)
    Δδrot1_u = np.arctan2((X_u - X_uprev), (Y_u - Y_uprev)) - θ_uprev
    Δδrot2_u = θ_u - θ_uprev - Δδrot1_u
    
    Δδtr_u=Δδtr_u+np.random.normal(0, 0.02)#0.01
    Δδrot1_u=Δδrot1_u+np.random.normal(0, 0.002)#0.005
    Δδrot2_u=Δδrot2_u+np.random.normal(0, 0.001)#0.005
    # Noise Parameters
    α1, α2, α3, α4 = 0.000001, 0.000001, 0.000001, 0.000001

    # Samples of the actual differences
    Δδtr = Δδtr_u + np.random.normal(0,  α1* np.abs(Δδrot1_u) + α2* np.abs(Δδtr_u) )
    Δδrot1 = Δδrot1_u + np.random.normal(0,  α3* np.abs(Δδtr_u) + α4* (np.abs(Δδrot1_u) + np.abs(Δδrot2_u)) )
    Δδrot2 = Δδrot2_u + np.random.normal(0,  α1* np.abs(Δδrot2_u) + α2* np.abs(Δδtr_u) )
    
    # Converting the sample differences into sample state (new particle)
    X_p = X + Δδtr * np.sin(θ + Δδrot1)
    Y_p = Y + Δδtr * np.cos(θ + Δδrot1)
    Θ_p = θ + Δδrot1 + Δδrot2
    
    # Sampled Particle
    return (X_p, Y_p, Θ_p)


def project(z, xₜ, yₜ, θₜ):
  xₑ, yₑ = np.zeros(NUM_RAYS), np.zeros(NUM_RAYS)
  i_0, j_0 = location_to_grid(xₜ, yₜ)
  for index, (angle, distance) in enumerate(zip(np.arange(len(z.ranges)) * z.angle_increment, z.ranges)):
    if distance < z.range_max:
      # Systematic error correction
      if angle > PI/2 -0.004 and angle < 3/2 * PI-0.005:
        j=j_0-int(SENSOR_DIST*np.sin(θₜ+PI/4)/RESOLUTION)
        i=i_0+int(SENSOR_DIST*np.cos(θₜ+PI/4)/RESOLUTION)
      else:
        j=j_0+int(SENSOR_DIST*np.sin(θₜ+PI/4)/RESOLUTION)
        i=i_0-int(SENSOR_DIST*np.cos(θₜ+PI/4)/RESOLUTION)

      θ = -θₜ + angle + 135/180 * PI
      iₑ=i + int(distance / RESOLUTION * np.cos(θ))
      jₑ=j + int(distance / RESOLUTION * np.sin(θ))
      xₑ[index], yₑ[index] = grid_to_location(iₑ, jₑ)

    xₑ[xₑ < -20 ], xₑ[xₑ >= 28 ]  = -np.inf, -np.inf
    yₑ[yₑ < -16 ], yₑ[yₑ >= 34 ] = -np.inf, -np.inf
    return xₑ, yₑ


def MCL(u, z):
    global X_prev
    global M
    X = np.zeros((PARTICLES_NUM,3))# 3 for pose(x,y,theta) {particle array}
    W = np.zeros(PARTICLES_NUM)
    # For each particle
    for i in range(PARTICLES_NUM):
      X[i] = sample_motion_model(X_prev[i], u)
      M[i] = update_map(X[i], z, M[i])
      W[i] = p_sensor_model(X[i], z, M[i])
    # Resample X according to W
    #print(W)
    r = np.arange(len(X_prev))
    r = np.random.choice(r, size=len(X_prev), p=W/np.sum(W))
    X_prev = X[r]
    M = M[r]
    # return the particle with the highest weight
    best_particle_index = np.argmax(W)
    return X[best_particle_index], M[best_particle_index]

def sensor_data_callback(data):
  global pose_prev_truth
  global current_pos_estimite
  global current_map_estimite
  # Get the odometry and laser scan data
  odometry = data.odometry
  z_t = data.laser_scan
  x, y = odometry.pose.pose.position.x, odometry.pose.pose.position.y # Location of the robot
  orientation = odometry.pose.pose.orientation
  Θ = np.arctan2(2 * (orientation.w * orientation.z), 1 - 2 * (orientation.z * orientation.z)) # Orientation of the robot
  
  pose_truth = (x, y, Θ)
  u_t = (pose_prev_truth, pose_truth)
  
  pose_prev_truth = pose_truth
  
  current_pos_estimite, current_map_estimite = MCL(u_t, z_t)
  
  # Publish the estimated pose and particles
  # print(current_pos_estimite)
  pose = Pose()
  pose.position.x=current_pos_estimite[0]
  pose.position.y=current_pos_estimite[1]
  pose.position.z=0
  pose.orientation.x=0
  pose.orientation.y=0
  pose.orientation.w=1
  pose.orientation.z=np.sin(current_pos_estimite[2]/2)
  
  estimated_pose_particle = PoseArray()
  estimated_pose_particle.poses=[]
  estimated_pose_particle.header.frame_id = "robot_map"
  estimated_pose_particle.poses.append(pose)
  pose_publisher.publish(estimated_pose_particle)
  
  particles = PoseArray()
  particles.poses=[]
  particles.header.frame_id = "robot_map"
  poses = []
  for i in range(PARTICLES_NUM):
    particle = Pose()
    particle.position.x = X_prev[i][0]
    particle.position.y = X_prev[i][1]
    particle.position.z = 0
    particle.orientation.x = 0
    particle.orientation.y = 0
    particle.orientation.w = 1
    particle.orientation.z = np.sin(X_prev[i][2]/2)
    poses.append(particle)
    
  particles.poses = poses
  particle_publisher.publish(particles)
  os.system('clear')
  
  print("true pose: ", (round(pose_truth[0],2), round(pose_truth[1],2), round(pose_truth[2],2)))
  print("estimated pose: ", (round(current_pos_estimite[0],2), round(current_pos_estimite[1],2), round(current_pos_estimite[2],2)))
  publish_log_odds_occupancy_grid(current_map_estimite)


if __name__ == '__main__':
  rospy.init_node('MCL')
  rate = rospy.Rate(30)
  pose_publisher = rospy.Publisher('/estimate_pose', PoseArray)
  map_publisher = rospy.Publisher('/estimate_map', OccupancyGrid)
  particle_publisher = rospy.Publisher('/particles', PoseArray)
  sensor_data_subscriber = rospy.Subscriber('/aligned_sensors', SensorData, sensor_data_callback, queue_size=1)
  rospy.spin()
