#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from cmp_msgs.msg import SensorData

# Initial Pose
X_OFFSET = 50
Y_OFFSET = 50

# Map Parameters
RESOLUTION = 0.1
WIDTH = 99.84
HEIGHT = 99.84
MAP_SIZE_HEIGHT = int(HEIGHT / RESOLUTION)
MAP_SIZE_WIDTH = int(WIDTH / RESOLUTION)

location_to_grid = lambda x, y: (int((y + X_OFFSET)/ RESOLUTION), int((x + Y_OFFSET)/ RESOLUTION))
grid_to_location = lambda i, j: (j * RESOLUTION - X_OFFSET, i * RESOLUTION - Y_OFFSET)

I_uprev, J_uprev = location_to_grid(X_OFFSET, Y_OFFSET)
pose_uprev = [X_OFFSET, Y_OFFSET, 0]

NUM_RAYS = 360

def sensor_data_callback(data):
  # Get the odometry and laser scan data
  odometry = data.odometry
  laser_scan = data.laser_scan

  # Get the robot's current and previous pose via odometry
  x, y = odometry.pose.pose.position.x, odometry.pose.pose.position.y # Location of the robot
  orientation = odometry.pose.pose.orientation
  Θ = np.arctan2(2 * (orientation.w * orientation.z), 1 - 2 * (orientation.z * orientation.z)) # Orientation of the robot
  
  # Make the current and previous pose into an action
  I_u, J_u = location_to_grid(x, y)
  pose_u = [I_u, J_u, Θ]                                          # Current pose by u (odometry)         
  u_t = (pose_uprev, pose_u)                                          # This timestep's action (to be used in PF)   
  
  pose_uprev = pose_u
  
  
  # Will be used in particle filter (x_prev will be a particle and u_t will be the action)
  def sample_motion_model(pose_prev, u_t):
    # Previous state 
    I, J, θ = pose_prev
    # Action
    pose_uprev, pose_u = u_t
    I_u, J_u, θ_u = pose_u
    I_uprev, J_uprev, θ_uprev = pose_uprev
    
    # Difference due to odometry
    Δδtr_u = np.sqrt((I_u - I_uprev)**2 + (J_u - J_uprev)**2)
    Δδrot1_u = np.arctan2((J_u - J_uprev), (I_u - I_uprev)) - θ_uprev
    Δδrot2_u = θ_u - θ_uprev - Δδrot1_u
    
    # Noise Parameters
    α1, α2, α3, α4 = 0.1, 0.1, 0.1, 0.1
    
    # Samples of the actual differences
    Δδtr = Δδtr_u + np.random.normal(0,  α1* Δδrot1_u + α2* Δδtr_u )
    Δδrot1 = Δδrot1_u + np.random.normal(0,  α3* Δδtr_u + α4* (Δδrot1_u + Δδrot2_u) )
    Δδrot2 = Δδrot2_u + np.random.normal(0,  α1* Δδrot2_u + α2* Δδtr_u )
    
    # Converting the sample differences into sample state (new particle)
    I_p = I + Δδtr * np.sin(θ + Δδrot1)
    J_p = J + Δδtr * np.cos(θ + Δδrot1)
    Θ_p = θ + Δδrot1 + Δδrot2
    
    # Sampled Particle
    return (I_p, J_p, Θ_p)
  
  
## Will be used to model the sensor 
  def project(z, xₜ, yₜ, θₜ):
    xₑ = np.rint(xₜ + z* np.cos(np.deg2rad(np.arange(θₜ, θₜ + NUM_RAYS, 1)))).astype(int)
    xₑ[xₑ < -20 ] = -np.inf
    xₑ[xₑ >= 28 ] = -np.inf
    yₑ = np.rint(yₜ + z * np.sin(np.deg2rad(np.arange(θₜ, θₜ + NUM_RAYS, 1)))).astype(int)
    yₑ[yₑ < -16 ] = -np.inf
    yₑ[yₑ >= 34 ] = -np.inf
    return xₑ, yₑ
  
  ## Will be used in particle filter (x is the particle, z is the measurements and m is the map)
  def p_sensor_model(x, z, m):
    # Transform the map into a likelihood field
    occupancy_grid = np.round(m)                    # Binary map
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

    
  
  # Particle Filter
  
  PARTICLES_NUM = 1000 # Number of particles

  # Generate random x, y, Θ values within a window of size 5 around the robot (initially)
  X_prevx = np.random.uniform(X_OFFSET - 2.5, X_OFFSET + 2.5, size=(PARTICLES_NUM, 1))
  Y_prevy = np.random.uniform(Y_OFFSET - 2.5, Y_OFFSET + 2.5, size=(PARTICLES_NUM, 1))
  Θ_prevth = np.random.uniform(0 , 360, size=(PARTICLES_NUM, 1))
  X_prev = np.hstack((X_prevx, Y_prevy, Θ_prevth))
  
  def MCL(u, z, m):
    X = np.zeros((PARTICLES_NUM,3))
    W = np.zeros((PARTICLES_NUM,1))
    # For each particle
    for i in range(PARTICLES_NUM):
      X[i] = sample_motion_model(X_prev[i], u)
      W[i] = p_sensor_model(X[i], z, m)
    # Resample X according to W
    X_prev = np.random.choice(X, p=W, k=PARTICLES_NUM)
    # return the particle with the highest weight
    return X[np.argmax(W)]
  

if __name__ == '__main__':
  rospy.init_node('MCL')
  rate = rospy.Rate(30)
  pose_publisher = rospy.Publisher('/pose', Pose)
  sensor_data_subscriber = rospy.Subscriber('/aligned_sensors', SensorData, sensor_data_callback)
  rospy.spin()