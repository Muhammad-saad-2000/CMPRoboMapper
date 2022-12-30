#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from cmp_msgs.msg import SensorData

# Initial Pose
X_OFFSET = 50/1.5
Y_OFFSET = 50/1.5

# Map Parameters
RESOLUTION = 0.1
WIDTH=99.84/1.5
HEIGHT=99.84/1.5
MAP_SIZE_HEIGHT = int(HEIGHT / RESOLUTION)
MAP_SIZE_WIDTH = int(WIDTH / RESOLUTION)

location_to_grid = lambda x, y: (int((x + X_OFFSET)/ RESOLUTION), int((y + Y_OFFSET)/ RESOLUTION))
grid_to_location = lambda i, j: (i * RESOLUTION - X_OFFSET, j * RESOLUTION - Y_OFFSET)

I_uprev, J_uprev = location_to_grid(Y_OFFSET, X_OFFSET)
pose_uprev = [X_OFFSET, Y_OFFSET, 0]

NUM_RAYS = 360

def sensor_data_callback(data):
  # Get the odometry and laser scan data
  odometry = data.odometry
  laser_scan = data.laser_scan
  Δt = odometry.header.stamp.to_sec() - odometry.header.stamp.to_sec()

  # Get the robot's current and previous pose via odometry
  x, y = odometry.pose.pose.position.x, odometry.pose.pose.position.y # Location of the robot
  orientation = odometry.pose.pose.orientation
  Θ = np.arctan2(2 * (orientation.w * orientation.z), 1 - 2 * (orientation.z * orientation.z)) # Orientation of the robot
  
  # Make the current and previous pose into an action
  I_u, J_u = location_to_grid(y, x)
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
    xₑ[xₑ < 0 ] = -1
    xₑ[xₑ >= MAP_SIZE_WIDTH ] = -1
    yₑ = np.rint(yₜ + z * np.sin(np.deg2rad(np.arange(θₜ, θₜ + NUM_RAYS, 1)))).astype(int)
    yₑ[yₑ < 0 ] = -1
    yₑ[yₑ >= MAP_SIZE_HEIGHT ] = -1
    return xₑ, yₑ
  
  ## Will be used in particle filter (x is the particle, z is the measurements and m is the map)
  def p_sensor_model(x, z, m):
    # Transform the map into a likelihood field
    occupancy_grid = np.round(m)                    # Binary map
    ll_field = cv2.distanceTransform(occupancy_grid, cv2.DIST_L2, 0)
    σ, π = 6.25, 3.14
    ll_field = np.array(ll_field)
    ll_field = 1/np.sqrt(2*π*σ) * np.exp(-0.5*((ll_field)/σ)**2)
    
    # Modify the likelihood field to account for random noise
    norm = np.max(ll_field)
    ll_field = (0.9 * ll_field + 0.1 * 1/(3000))/norm      
    # To be added in case of max range reading    
    max_range_weight = 0.09 * 1/(10 * norm)
    MAX_RANGE = 300
    
    xt, yt, θt = x
    
    xₑ, yₑ = project(z, xt, yt, θt)
    p = 1
    for i, (xb, yb) in enumerate(zip(xₑ, yₑ)):
        if xb == -1 or yb == -1:                       # The ray has crossed the boundary of the map
          p *= 10e-3
        else:
          p *= ll_field[xb, yb] + (max_range_weight if MAX_RANGE-10 < z[i] <=MAX_RANGE else 0)

    return p

    
  
  # Particle Filter
  
  J = 1000 # Number of particles

  # Generate random x, y, Θ values within a window of size 100 around the robot
  X_prevx = np.random.uniform(X_OFFSET - 50, X_OFFSET + 50, size=(J, 1))
  Y_prevy = np.random.uniform(Y_OFFSET - 50, Y_OFFSET + 50, size=(J, 1))
  Θ_prevth = np.random.uniform(0 , 360, size=(J, 1))
  X_prev = np.hstack((X_prevx, Y_prevy, Θ_prevth))
  
  def MCL(u, z, m):
    X = np.zeros((J,3))
    W = np.zeros((J,1))
    # For each particle
    for i in range(J):
      X[i] = sample_motion_model(X_prev[i], u)
      W[i] = p_sensor_model(X[i], z, m)
    # Resample X according to W
    X_prev = np.random.choice(X, p=W, k=J)
    # return the particle with the highest weight
    return X[np.argmax(W)]
  

if __name__ == '__main__':
  rospy.init_node('MCL')
  rate = rospy.Rate(30)
  pose_publisher = rospy.Publisher('/pose', Pose)
  sensor_data_subscriber = rospy.Subscriber('/aligned_sensors', SensorData, sensor_data_callback)
  rospy.spin()