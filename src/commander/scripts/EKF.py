#!/usr/bin/env pyₜhon3
import rospy
import numpy as np
import cv2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from cmp_msgs.msg import SensorData
pi, sqrt, absl, det, exp, inv, sin, cos = np.pi,  np.sqrt, np.abs, np.linalg.det, np.exp, np.linalg.inv, np.sin, np.cos


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
MAX_RANGE = 10


## For Mapping

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
  
  x, y, θ = x
  i_0, j_0 = location_to_grid(x, y)

  ### NEEDS UPDATE
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
  occupancy_grid = 1 / (1 + np.exp(-log_occupancy_grid))
  return occupancy_grid



    
## For the Sensor Model (Scan Matching)

def project(z, xₜ, yₜ, θₜ):
    xₑ = np.rint(xₜ + z* cos(np.deg2rad(np.arange(θₜ, θₜ + NUM_RAYS, 1)))).astype(int)
    xₑ[xₑ < -20 ], xₑ[xₑ >= 28 ]  = -np.inf, -np.inf
    yₑ = np.rint(yₜ + z * sin(np.deg2rad(np.arange(θₜ, θₜ + NUM_RAYS, 1)))).astype(int)
    yₑ[yₑ < -16 ], yₑ[yₑ >= 34 ] = -np.inf, -np.inf
    return xₑ, yₑ
  
def p_sensor_model(x, z, m):
  # Transform the map into a likelihood field
  occupancy_grid = np.round(m)                    # Binary map
  ll_field = cv2.distanceTransform(occupancy_grid, cv2.DIST_L2, 0)
  σ, π = 20, 3.14
  ll_field = np.array(ll_field)
  ll_field = 1/sqrt(2*π*σ) * exp(-0.5*((ll_field)/σ)**2)
  
  # Modify the likelihood field to account for random noise
  norm = np.max(ll_field)
  ll_field = (0.991 * ll_field)/norm
  # To be added in case of max range reading
  max_range_weight = 0.009 * 1/((30-27))
  MAX_RANGE = 30
  
  xₜ, yₜ, θₜ = x
  
  xₑ, yₑ = project(z, xₜ, yₜ, θₜ)
  p = 1
  for i, (xb, yb) in enumerate(zip(xₑ, yₑ)):
      if xb == -np.inf or yb == -np.inf:# The ray has crossed the boundary of the map
        p *= 10e-3
      else:
        p *= ll_field[xb, yb] + (max_range_weight if MAX_RANGE-0.5 < z[i] <= MAX_RANGE else 0)

  return p
  



def sensor_data_callback(data):
  # Get the odometry and laser scan data
  odometry = data.odometry
  laser_scan = data.laser_scan

  # Get the robot's current and previous pose via odometry
  xₜ, yₜ = odometry.pose.pose.position.x, odometry.pose.pose.position.y # Location of the robot
  orientation = odometry.pose.pose.orientation
  θₜ = np.arctan2(2 * (orientation.w * orientation.z), 1 - 2 * (orientation.z * orientation.z)) # Orientation of the robot
  
  vx, vy , = odometry.twist.twist.linear.x, odometry.twist.twist.linear.y
  wx, wy = odometry.twist.twist.angular.x, odometry.twist.twist.angular.y
  vₜ  = sqrt(vx**2 + vy**2)
  ωₜ   = sqrt(wx**2 + wy**2)
  uₜ = (vₜ, ωₜ)
  Δt = odometry.header.stamp.secs - odometry.header.stamp.nsecs * 1e-9
  
  μ = np.array([xₜ, yₜ, θₜ])
  Σ = np.array([[0.001, 0, 0], [0, 0.001, 0], [0, 0, 0.001]]) 
  

  def Pu(x, y, θ, Σₚ, μₚ):
    norm = 1/((2 * pi)**1.5 *sqrt(absl(det(Σₚ))))
    Xu = μₚ - np.array([x, y, θ])
    return norm * exp(-0.5 * (Xu) @ inv(Σₚ) @ (Xu).T)
    
    
    
  def EKF_Localization(μ , Σ , uₜ, zₜ, mₜ):
  
  ### Prediction Step
  
    θₚ = μ[2]
    
    vₜ, ωₜ = uₜ
    rₜ = np.abs(vₜ/ωₜ)
    
    # Motion model Jacobians
    Gₜ = np.array([
      [1, 0, -rₜ * cos(θₚ) + rₜ * cos(θₚ + ωₜ * Δt)], 
      [0, 1, -rₜ * sin(θₚ) + rₜ * sin(θₚ + ωₜ * Δt)], 
      [0, 0, 1                                   ]
      ])
    
    Vₜ = np.array([
      [(-sin(θₚ) + sin(θₚ + ωₜ * Δt))/ωₜ, rₜ * (sin(θₚ) - sin(θₚ + ωₜ * Δt)) / ωₜ + rₜ * cos(θₚ + ωₜ * Δt) * Δt],
      [(cos(θₚ) - cos(θₚ + ωₜ * Δt))/ωₜ, -rₜ * (cos(θₚ) - cos(θₚ + ωₜ * Δt)) / ωₜ + rₜ * sin(θₚ + ωₜ * Δt) * Δt],
      [0                               , Δt                                                               ]            
    ])
    
    
    # Motion Noise Parameters & Covariance
    α1, α2, α3, α4 = 0.001, 0.001, 0.001, 0.001 
    Mₜ = np.array([
      [α1 * vₜ**2 + α2 * ωₜ**2, 0],
      [0, α3 * vₜ**2 + α4 * ωₜ**2]
    ])
    
    # Predicted State & Covariance
    μₚ = μ + np.array([
      [-rₜ * sin(θₚ) + rₜ * sin(θₚ + ωₜ * Δt)],
      [ rₜ * cos(θₚ) - rₜ * cos(θₚ + ωₜ * Δt)],
      [ωₜ * Δt                             ]
    ])
    
    Σₚ = Gₜ @ Σ @ Gₜ.T + Vₜ @ Mₜ @ Vₜ.T
    
    
    ### Mapping Step
    # Inputs are the known pose (xm, ym, θm) and the sensor readings (zₜ)
   
    # Required is a map update for mₜ
    mₜ = update_map(μₚ, zₜ, mₜ)
    
    
    ### Correction Step via Scan Matching
    
    xm, ym, θm = μₚ[0], μₚ[1], μₚ[2]              
    σx, σy, σθ = Σ[0,0], Σ[1,1], Σ[2,2]
    
    global X
    X = (None, None, None)
    Pmax = 0
    for x in range(xm - 1.5*σx, xm + 1.5*σx, 0.5):
      for y in range(ym - 1.5*σy, ym + 1.5*σy, 0.5):
        for θ in range(θm - 1.5*σθ, θm + 1.5*σθ, 0.5):
          p = Pu(x, y, θ, Σₚ, μₚ) * p_sensor_model((x, y, θ), zₜ, mₜ)
          if p > Pmax:
            Pmax = p
            X = (x, y, θ)
    
    μc = np.array(X)
    Σc = Σₚ   
    
    return μc, Σc 
    
    
        

                    
                    
    
  
  
  
  

  

if __name__ == '__main__':
  rospy.init_node('MCL')
  rate = rospy.Rate(30)
  pose_publisher = rospy.Publisher('/pose', Pose)
  sensor_data_subscriber = rospy.Subscriber('/aligned_sensors', SensorData, sensor_data_callback)
  rospy.spin()