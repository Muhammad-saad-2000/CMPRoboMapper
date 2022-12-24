#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys
import select
import tty
import termios


def isData():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])


if __name__ == '__main__':
  rospy.init_node("keyboard_controller")
  rate = rospy.Rate(60)
  pubObj = rospy.Publisher("/robot/robotnik_base_control/cmd_vel", Twist)
  
  speed = 0
  speed_turn = 0
  top_speed = 2
  top_speed_turn = 2
  increment = 0.8
  old_settings = termios.tcgetattr(sys.stdin)
  try:
    tty.setcbreak(sys.stdin.fileno())
    while not rospy.is_shutdown():
      if isData():
        c = sys.stdin.read(1)
        if c == 'w':
          speed += (top_speed-speed)*increment
        elif c == 's':
          speed += (-top_speed-speed)*increment
        elif c == 'a':
          speed_turn += (top_speed_turn-speed_turn)*increment
        elif c == 'd':
          speed_turn += (-top_speed_turn-speed_turn)*increment
      else:
        speed += (-speed)*increment
        speed_turn += (-speed_turn)*increment
  
      twist = Twist()
      twist.linear.x = speed
      twist.angular.z = speed_turn
      pubObj.publish(twist)
      rate.sleep()
  finally:
      termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
