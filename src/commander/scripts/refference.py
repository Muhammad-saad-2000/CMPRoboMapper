#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry

def callback(data):
  pubObj.publish(data)

if __name__ == '__main__':
  rospy.init_node("Rviz_Test")# Node name (could get from rosnode list)
  rospy.loginfo("Rviz_Test node has been initialized") # Log message (could get from rosnode echo /Rviz_Test)
  
  rate = rospy.Rate(30)
  
  pubObj = rospy.Publisher("simple_Rviz", Odometry) # Topic name (could get from rostopic list)
  
  subObj = rospy.Subscriber("odom", Odometry, callback)
  
  # NOTE:Only one of the two lines below is needed
  # for the subscriber to work
  rospy.spin()
  # for the publisher to work
  while not rospy.is_shutdown():
    rate.sleep()