#!/usr/bin/env python

'''
Main functions
'''
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import numpy as np
import rospy
import sys
from easydict import EasyDict


# Robot motion commands:
# http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
from geometry_msgs.msg import Twist
# Laser scan message:
# http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
# For groundtruth information.
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion

from laser_scanner import SimpleLaser
from robotics_control import Police, Baddy
from global_positioning import GroundtruthPose


def run(config):
  rospy.init_node('obstacle_avoidance')
  # avoidance_method = globals()[args.mode]

  # Update control every 100 ms.
  rate_limiter = rospy.Rate(100)

  publishers = []
  lasers = []

  robot_intances = []
  for robot_index, robot in enumerate(config.robots):

    print(robot)
    publisher = rospy.Publisher('/{}/cmd_vel'.format(robot.name), Twist, queue_size=5)

    if robot.type == 'Police':
      RobotClass = Police
      PositioningClass = GroundtruthPose
      SensorClass = SimpleLaser
    else:
      RobotClass = Baddy
      PositioningClass = GroundtruthPose
      SensorClass = SimpleLaser

    sensor = SensorClass(robot)
    positioning = PositioningClass(robot)
    robot_instance = RobotClass(publisher, config, robot, sensor, positioning)
    robot_intances.append(robot_instance)


  while not rospy.is_shutdown():
    # Make sure all measurements are ready.
    if not all([robot.sensor.ready for robot in robot_intances]):
      rate_limiter.sleep()
      continue
    if not all([robot.positioning.ready for robot in robot_intances]):
      rate_limiter.sleep()
      continue

    for robot in robot_intances:
      robot.action()
      print(robot.get_current_position())



if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='Runs obstacle avoidance')
  parser.add_argument('--mode', action='store', default='braitenberg', help='Method.', choices=['braitenberg', 'rule_based'])
  args, unknown = parser.parse_known_args()

  print(args)
  print ("Python Version {}".format(str(sys.version).replace('\n', '')))

  configs = {
    'robots':[
      {
        'name': 'robot1',
       'type': 'police',
      },
      {
        'name': 'robot2',
        'type': 'police',
      },
      {
        'name': 'robot3',
        'type': 'baddy',
      },
      {
        'name': 'robot4',
        'type': 'baddy',
      },
    ]
  }

  configs = EasyDict(configs)
  configs.num_robots = len(configs.robots)

  print(configs)

  try:
    run(configs)
  except rospy.ROSInterruptException:
    pass
