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
pi, sqrt, absl, det, exp, inv, sin, cos = np.pi,  np.sqrt, np.abs, np.linalg.det, np.exp, np.linalg.inv, np.sin, np.cos


# Initial Pose
X_OFFSET = 50
Y_OFFSET = 50
# Map Parameters
RESOLUTION = 0.1
OCCUPAIED_AT_END = 1
OCCUPAIED_LOG_ODD= 1.018
UNOCCUPAIED_LOG_ODD = -1.018
WIDTH = 99.84
HEIGHT = 99.84
MAP_SIZE_HEIGHT = int(HEIGHT / RESOLUTION)
MAP_SIZE_WIDTH = int(WIDTH / RESOLUTION)
SENSOR_DIST=np.sqrt(2)/5+0.1
MAX_RANGE = 30
NUM_RAYS = 720
ϵ = 0.000001
is_init_state = True
μc=None 
Σc=None

location_to_grid = lambda x, y: (int((y + X_OFFSET)/ RESOLUTION), int((x + Y_OFFSET)/ RESOLUTION))
grid_to_location = lambda i, j: (j * RESOLUTION - X_OFFSET, i * RESOLUTION - Y_OFFSET)

I_uprev, J_uprev = location_to_grid(X_OFFSET, Y_OFFSET)
pose_uprev = [X_OFFSET, Y_OFFSET, 0]


m = np.zeros((MAP_SIZE_WIDTH, MAP_SIZE_HEIGHT)) #log odds map

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
  occupancy_grid_publisher.publish(occupancy_grid_msg)

def publish_estimated_pose(μ):
  x, y, θ = μ
  pose_temp = Pose()
  pose_temp.position.x=x
  pose_temp.position.y=y
  pose_temp.position.z=0
  pose_temp.orientation.x=0
  pose_temp.orientation.y=0
  pose_temp.orientation.w=1
  pose_temp.orientation.z=np.sin(θ/2)
  
  estimated_pose_particle = PoseArray()
  estimated_pose_particle.poses=[]
  estimated_pose_particle.header.frame_id = "robot_map"
  estimated_pose_particle.poses.append(pose_temp)
  estimate_pose_publisher.publish(estimated_pose_particle)

## For Mapping
def location_to_grid(x, y):
  i=int((y + X_OFFSET)/ RESOLUTION)
  j=int((x + Y_OFFSET)/ RESOLUTION)
  if i>=MAP_SIZE_WIDTH:
    i=MAP_SIZE_WIDTH-1
  if j>=MAP_SIZE_HEIGHT:
    j=MAP_SIZE_HEIGHT-1
  return i, j

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


def update_map(x, z, m):
  log_occupancy_grid = m
  laser_scan = z
  
  xₚ = x[0,0]
  yₚ = x[1,0]
  θₚ = x[2,0]
  i_0, j_0 = location_to_grid(xₚ, yₚ)
  for angle, distance in zip(np.arange(len(laser_scan.ranges)) * laser_scan.angle_increment, laser_scan.ranges):
    if distance < laser_scan.range_max:
      # Systematic error correction
      if angle > pi/2-0.004 and angle < 3/2 * pi-0.005:
        j=j_0-int(SENSOR_DIST*np.sin(θₚ+pi/4)/RESOLUTION)
        i=i_0+int(SENSOR_DIST*np.cos(θₚ+pi/4)/RESOLUTION)
      else:
        j=j_0+int(SENSOR_DIST*np.sin(θₚ+pi/4)/RESOLUTION)
        i=i_0-int(SENSOR_DIST*np.cos(θₚ+pi/4)/RESOLUTION)
      points = free_grid_cells(i, j, -θₚ + angle + 135/180 * pi, distance)
      if len(points) > OCCUPAIED_AT_END:
        for point in points[:-OCCUPAIED_AT_END]:
          log_occupancy_grid[point] += UNOCCUPAIED_LOG_ODD
        for point in points[-OCCUPAIED_AT_END:]:
          log_occupancy_grid[point] += OCCUPAIED_LOG_ODD

  log_occupancy_grid[log_occupancy_grid > 20] = 20
  log_occupancy_grid[log_occupancy_grid < -20] = -20
  return log_occupancy_grid



## For the Sensor Model (Scan Matching)

def project(z, xₜ, yₜ, θₜ):
  xₑ, yₑ = np.zeros(NUM_RAYS), np.zeros(NUM_RAYS)
  i_0, j_0 = location_to_grid(xₜ, yₜ)
  for index, (angle, distance) in enumerate(zip(np.arange(len(z.ranges)) * z.angle_increment, z.ranges)):
    if distance < z.range_max:
      # Systematic error correction
      if angle > pi/2 -0.004 and angle < 3/2 * pi-0.005:
        j=j_0-int(SENSOR_DIST*np.sin(θₜ+pi/4)/RESOLUTION)
        i=i_0+int(SENSOR_DIST*np.cos(θₜ+pi/4)/RESOLUTION)
      else:
        j=j_0+int(SENSOR_DIST*np.sin(θₜ+pi/4)/RESOLUTION)
        i=i_0-int(SENSOR_DIST*np.cos(θₜ+pi/4)/RESOLUTION)

      θ = -θₜ + angle + 135/180 * pi
      iₑ=i + int(distance / RESOLUTION * np.cos(θ))
      jₑ=j + int(distance / RESOLUTION * np.sin(θ))
      xₑ[index], yₑ[index] = grid_to_location(iₑ, jₑ)

    xₑ[xₑ < -20 ], xₑ[xₑ >= 28 ]  = -np.inf, -np.inf
    yₑ[yₑ < -16 ], yₑ[yₑ >= 34 ] = -np.inf, -np.inf
    return xₑ, yₑ


def p_sensor_model(x, z, m):
  occupancy_grid = np.round(1 / (1 + np.exp(-m))) # Binary map (occupancy grid)
  occupancy_grid = occupancy_grid.astype(np.uint8) * 255
  # Transform the map into a likelihood field
  ll_field = cv2.distanceTransform(occupancy_grid, cv2.DIST_L2, 0)
  σ, π = 100, 3.14
  ll_field = np.array(ll_field)
  ll_field = 1/np.sqrt(2*π*σ) * np.exp(-0.5*((ll_field)/σ)**2)
  
  # Modify the likelihood field to account for random noise
  norm = np.max(ll_field)
  ll_field = (0.991 * ll_field)/norm
  # To be added in case of max range reading
  max_range_weight = 0.009 * 1/((30-27) * norm)
  
  xt, yt, θt = x
  
  xₑ, yₑ = project(z, xt, yt, θt)
  p = 1
  for i, (xb, yb) in enumerate(zip(xₑ, yₑ)):
      if xb == -np.inf or yb == -np.inf:# The ray has crossed the boundary of the map
        p *= 10e-3
      else:
        ib, jb = location_to_grid(xb, yb)
        p *= ll_field[ib, jb] + (max_range_weight if MAX_RANGE-0.5 < z.ranges[i] <= MAX_RANGE else 0)
  return p

def Pu(x, y, θ, Σₚ, μₚ):
    Σₚ=Σₚ.astype(float)
    μₚ=μₚ.reshape(1,3)
    norm = 1/((2 * pi)**1.5 *sqrt(absl(det(Σₚ))))
    Xu = μₚ - np.array([x, y, θ])
    Xu = Xu.astype(float)
    return norm * exp(-0.5 * (Xu) @ inv(Σₚ) @ (Xu).T)

def EKF_Localization(μ , Σ , uₜ, zₜ, mₜ, Δt):
    ### Prediction Step
    θₚ = μ[2]
    
    vₜ, ωₜ = uₜ
    rₜ = np.abs(vₜ/(ωₜ+ϵ))
    
    # Motion model Jacobians
    Gₜ = np.array([
      [1, 0, -rₜ * cos(θₚ) + rₜ * cos(θₚ + ωₜ * Δt)], 
      [0, 1, -rₜ * sin(θₚ) + rₜ * sin(θₚ + ωₜ * Δt)], 
      [0, 0, 1                                   ]
      ])
    
    Vₜ = np.array([
      [(-sin(θₚ) + sin(θₚ + ωₜ * Δt))/(ωₜ+ϵ), rₜ * (sin(θₚ) - sin(θₚ + ωₜ * Δt)) / (ωₜ+ϵ) + rₜ * cos(θₚ + ωₜ * Δt) * Δt],
      [(cos(θₚ) - cos(θₚ + ωₜ * Δt))/(ωₜ+ϵ), -rₜ * (cos(θₚ) - cos(θₚ + ωₜ * Δt)) / (ωₜ+ϵ) + rₜ * sin(θₚ + ωₜ * Δt) * Δt],
      [0                               , Δt                                                               ]            
    ])
    
    
    # Motion Noise Parameters & Covariance
    α1, α2, α3, α4 = 1e-5, 1e-5, 1e-5, 1e-5 
    Mₜ = np.array([
      [α1 * vₜ**2 + α2 * ωₜ**2, 0],
      [0, α3 * vₜ**2 + α4 * ωₜ**2]
    ])
    
    # Predicted State & Covariance
    xm=(-rₜ * sin(θₚ) + rₜ * sin(θₚ + ωₜ * Δt))
    ym=(rₜ * cos(θₚ) - rₜ * cos(θₚ + ωₜ * Δt))
    xm=xm[0] if type(xm) is np.ndarray else xm
    ym=ym[0] if type(ym) is np.ndarray else ym
    
    μₚ = μ + np.array([
      [xm],
      [ym],
      [ωₜ * Δt]
    ])
    
    Σₚ = Gₜ @ Σ @ Gₜ.T + Vₜ @ Mₜ @ Vₜ.T
    
    
    ### Mapping Step
    # Inputs are the known pose (xm, ym, θm) and the sensor readings (zₜ)
   
    # Required is a map update for mₜ
    mₜ = update_map(μₚ, zₜ, mₜ)
    
    
    ### Correction Step via Scan Matching
    
    xm, ym, θm = μₚ[0], μₚ[1], μₚ[2]              
    σx, σy, σθ = Σ[0,0], Σ[1,1], Σ[2,2]
    X = (None, None, None)
    Pmax = 0
    for x in np.arange(xm - 0.1*σx, xm + 0.1*σx, 5e-4):
      for y in np.arange(ym - 0.1*σy, ym + 0.1*σy, 5e-4):
        for θ in np.arange(θm - 0.1*σθ, θm + 0.1*σθ, 5e-4):
          p = Pu(x, y, θ, Σₚ, μₚ) * p_sensor_model((x, y, θ), zₜ, mₜ)
          if p > Pmax:
            Pmax = p
            X = [[x], [y], [θ]]
    μc = np.array(X)
    Σc = Σₚ 
    return μc, Σc

def sensor_data_callback(data):
  global m
  global μc
  global Σc
  global is_init_state
  # Get the odometry and laser scan data
  odometry = data.odometry
  zₜ = data.laser_scan

  # Get the robot's current and previous pose via odometry
  xₜ, yₜ = odometry.pose.pose.position.x, odometry.pose.pose.position.y # Location of the robot
  orientation = odometry.pose.pose.orientation
  θₜ = np.arctan2(2 * (orientation.w * orientation.z), 1 - 2 * (orientation.z * orientation.z)) # Orientation of the robot
  
  vx, vy = odometry.twist.twist.linear.x, odometry.twist.twist.linear.y
  wx, wy = odometry.twist.twist.angular.x, odometry.twist.twist.angular.y
  vₜ  = sqrt(vx**2 + vy**2)
  ωₜ   = sqrt(wx**2 + wy**2)
  #print(vₜ, ωₜ)
  α1, α2, α3, α4 = 0.001, 0.001, 0.001, 0.001
  vₜ = vₜ + np.random.normal(0,α1 * vₜ**2 + α2 * ωₜ**2)
  ωₜ = ωₜ + np.random.normal(0,α3 * vₜ**2 + α4 * ωₜ**2)

  uₜ = (vₜ, ωₜ)
  Δt = (odometry.header.stamp.secs - odometry.header.stamp.nsecs) * 1e-9
  
  μ = np.array([[xₜ], [yₜ], [θₜ]])
  Σ = np.array([[0.001, 0, 0], [0, 0.001, 0], [0, 0, 0.001]]) 

  if is_init_state:
    μc, Σc = EKF_Localization(μ, Σ, uₜ, zₜ, m, Δt)
    is_init_state=False
  else:
    μc, Σc = EKF_Localization(μc, Σc, uₜ, zₜ, m, Δt)
  
  publish_log_odds_occupancy_grid(m)
  print(μc)
  publish_estimated_pose(μc)



if __name__ == '__main__':
  rospy.init_node('MCL')
  rate = rospy.Rate(30)
  estimate_pose_publisher = rospy.Publisher('/estimate_pose', PoseArray)
  occupancy_grid_publisher = rospy.Publisher('/occupancy_grid', OccupancyGrid)
  sensor_data_subscriber = rospy.Subscriber('/aligned_sensors', SensorData, sensor_data_callback, queue_size=5)
  rospy.spin()