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
PARTICLES_NUM = 2 # Number of particles
# Initial Pose
X_OFFSET = 50
Y_OFFSET = 50

# Map Parameters
RESOLUTION = 0.1
WIDTH = 99.84
HEIGHT = 99.84
MAP_SIZE_HEIGHT = int(HEIGHT / RESOLUTION)
MAP_SIZE_WIDTH = int(WIDTH / RESOLUTION)
# Initialize the particles
X_prev = np.hstack((np.random.uniform(X_OFFSET - 2.5, X_OFFSET + 2.5, size=(PARTICLES_NUM, 1)),
                    np.random.uniform(Y_OFFSET - 2.5, Y_OFFSET + 2.5, size=(PARTICLES_NUM, 1)),
                     np.random.uniform(0 , 360, size=(PARTICLES_NUM, 1))
                    ))


location_to_grid = lambda x, y: (int((y + X_OFFSET)/ RESOLUTION), int((x + Y_OFFSET)/ RESOLUTION))
grid_to_location = lambda i, j: (j * RESOLUTION - X_OFFSET, i * RESOLUTION - Y_OFFSET)

pose_prev_truth = [0, 0, 0]

NUM_RAYS = 720
ANGLE_INCREMENT = 0.0087266

current_pos_estimite=(0,0,0)

map = cv2.imread("src/commander/scripts/map.png", 0)

# Calculate the end points of the rays
# NOTE:z===laser_scan
def project(z, xₜ, yₜ, θₜ):
  x, y = xₜ, yₜ
  theta = θₜ
  xₑ, yₑ = np.zeros(NUM_RAYS), np.zeros(NUM_RAYS)
  i_0, j_0 = location_to_grid(x, y)
  for i, (angle, distance) in enumerate(zip(np.arange(len(z.ranges)) * z.angle_increment, z.ranges)):
    if distance < z.range_max:
      # Systematic error correction
      if angle > PI/2 and angle < 3/2 * PI-0.005:
        j=j_0-int(SENSOR_DIST*np.sin(theta+PI/4)/RESOLUTION)
        i=i_0+int(SENSOR_DIST*np.cos(theta+PI/4)/RESOLUTION)
      else:
        j=j_0+int(SENSOR_DIST*np.sin(theta+PI/4)/RESOLUTION)
        i=i_0-int(SENSOR_DIST*np.cos(theta+PI/4)/RESOLUTION)

      iₑ=i + int(distance / RESOLUTION * np.cos(angle))
      jₑ=j + int(distance / RESOLUTION * np.sin(angle))
      xₑ[i], yₑ[i] = grid_to_location(iₑ, jₑ)
  return xₑ, yₑ

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
    
    Δδtr_u=Δδtr_u+np.random.normal(0, 0.1)
    Δδrot1_u=Δδrot1_u+np.random.normal(0, 0.1)
    Δδrot2_u=Δδrot2_u+np.random.normal(0, 0.1)
    # Noise Parameters
    α1, α2, α3, α4 = 0.1, 0.1, 0.1, 0.1
    
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

## Will be used in particle filter (x is the particle, z is the measurements and m is the map)
# NOTE:z===laser_scan
# NOTE:m===(image array)
def p_sensor_model(x, z, m):
  # Transform the map into a likelihood field
  
  occupancy_grid = m                    # Binary map
  ll_field = cv2.distanceTransform(occupancy_grid, cv2.DIST_L2, 0)
  σ, π = 20, 3.14
  ll_field = np.array(ll_field)
  ll_field = 1/np.sqrt(2*π*σ) * np.exp(-0.5*((ll_field)/σ)**2)
  
  # Modify the likelihood field to account for random noise
  norm = np.max(ll_field)
  ll_field = (0.991 * ll_field)/norm
  # To be added in case of max range reading
  max_range_weight = 0.009 * 1/((30-27) * norm)
  MAX_RANGE = 30
  
  xt, yt, θt = x
  
  xₑ, yₑ = project(z, xt, yt, θt)
  p = 1
  for i, (xb, yb) in enumerate(zip(xₑ, yₑ)):
      if xb == -np.inf or yb == -np.inf:# The ray has crossed the boundary of the map
        p *= 10e-3
      else:
        p *= ll_field[xb, yb] + (max_range_weight if MAX_RANGE-0.5 < z[i] <= MAX_RANGE else 0)

  return p

# NOTE:z===laser_scan
# NOTE:m===(image array)
def MCL(u, z, m):
    global X_prev
    X = np.zeros((PARTICLES_NUM,3))# 3 for pose(x,y,theta) {particle array}
    W = np.zeros((PARTICLES_NUM,1))
    # For each particle
    for i in range(PARTICLES_NUM):
      X[i] = sample_motion_model(X_prev[i], u)
      W[i] = p_sensor_model(X[i], z, m)
    # Resample X according to W
    X_prev = np.random.choice(X, p=W, k=PARTICLES_NUM)
    # return the particle with the highest weight
    return X[np.argmax(W)]

def sensor_data_callback(data):
  global pose_prev_truth, map
  # Get the odometry and laser scan data
  odometry = data.odometry
  z_t = data.laser_scan
  x, y = odometry.pose.pose.position.x, odometry.pose.pose.position.y # Location of the robot
  orientation = odometry.pose.pose.orientation
  Θ = np.arctan2(2 * (orientation.w * orientation.z), 1 - 2 * (orientation.z * orientation.z)) # Orientation of the robot
  
  pose_truth=(x, y, Θ)
  u_t = (pose_prev_truth, pose_truth)
  
  pose_prev_truth = pose_truth
  
  current_pos_estimite = MCL(u_t, z_t, map)
  
  # Publish the estimated pose and particles
  pose = Pose()
  pose.position.x=current_pos_estimite[0]
  pose.position.y=current_pos_estimite[1]
  pose.position.z=0
  pose.orientation.x=0
  pose.orientation.y=0
  pose.orientation.w=1
  pose.orientation.z=np.sin(current_pos_estimite[2]/2)
  pose_publisher.publish(pose)
  
  particles = PoseArray()
  for i in range(PARTICLES_NUM):
    particle = Pose()
    particle.position.x = X_prev[i][0]
    particle.position.y = X_prev[i][1]
    particle.position.z = 0
    particle.orientation.x = 0
    particle.orientation.y = 0
    particle.orientation.w = 1
    particle.orientation.z = np.sin(X_prev[i][2]/2)
    particles.poses.append(particle)
  particle_publisher.publish(particles)


if __name__ == '__main__':
  rospy.init_node('MCL')
  rate = rospy.Rate(30)
  pose_publisher = rospy.Publisher('/estimate_pose', Pose)
  particle_publisher = rospy.Publisher('/particles', PoseArray)
  sensor_data_subscriber = rospy.Subscriber('/aligned_sensors', SensorData, sensor_data_callback)
  rospy.spin()
