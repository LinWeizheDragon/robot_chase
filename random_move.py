#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import numpy as np
import rospy
import sys
import math

NUM_ROBOTS=4

# Robot motion commands:
# http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
from geometry_msgs.msg import Twist
# Laser scan message:
# http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
from sensor_msgs.msg import LaserScan
# For groundtruth information.
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion

def avoid(distance):
  return max(0,0.5-np.exp(0.05-distance))

def braitenberg(front, front_left, front_right, left, right):
  u = 0.  # [m/s]
  w = 0.  # [rad/s] going counter-clockwise.
  #front=1 if math.isinf(front) else  front
  #front_left=1 if math.isinf(front_left) else front_left
  #front_right=1 if math.isinf(front_right) else  front_right
  #left=1 if math.isinf(left) else  left
  #right=1 if math.isinf(right) else  right
  #sys.stdout.write(str(front)+" "+str(left)+"\n")
  #sys.stdout.flush()
  vl=(avoid(right)+avoid(front_right))/2
  vr=(avoid(left)+avoid(front_left))/2
  
  u=avoid(front)#=(vr+vl)/2
  w=0.5*(vr-vl)+0.3*np.exp(-10*u)*np.sign(front_left-front_right)
  # MISSING: Implement a braitenberg controller that takes the range
  # measurements given in argument to steer the robot
  return u, w


def rule_based(front, front_left, front_right, left, right):
  u = 0.5  # [m/s]
  w = 0.  # [rad/s] going counter-clockwise.
  #sys.stdout.write(str(front>1)+" "+str(left>1)+"\n")
  #sys.stdout.flush()
  front=3 if math.isinf(front) else  front
  front_left=3 if math.isinf(front_left) else front_left
  front_right=3 if math.isinf(front_right) else  front_right
  left=3 if math.isinf(left) else  left
  right=3 if math.isinf(right) else  right
  
  if abs(front_left+left-front_right-right)>0.5 and (front_left<2 or front_right<2 or left<1 or right<1):
    w=(1.5*min(front_left,2)+min(left,1)-1.5*min(front_right,2)-min(right,1))*0.5
  # MISSING: Implement a rule-based controller that avoids obstacles.
  if front<0.8:
    u=max(front*0.5-0.2,0)
    sys.stdout.write(str(u)+"\n")
    sys.stdout.flush()
    if front_left>0.8 or front_right>0.8:
      w=0.2*max(front_left,front_right)*np.sign(front_left-front_right)
  return u, w


class SimpleLaser(object):
  def __init__(self,n_robot):
    rospy.Subscriber('/robot'+str(n_robot)+'/scan', LaserScan, self.callback)
    self._angles = [0., np.pi / 4., -np.pi / 4., np.pi / 2., -np.pi / 2.]
    self._width = np.pi / 180. * 10.  # 10 degrees cone of view.
    self._measurements = [float('inf')] * len(self._angles)
    self._indices = None
    self._n_robot=n_robot

  def callback(self, msg):
    # Helper for angles.
    def _within(x, a, b):
      pi2 = np.pi * 2.
      x %= pi2
      a %= pi2
      b %= pi2
      if a < b:
        return a <= x and x <= b
      return a <= x or x <= b;

    # Compute indices the first time.
    if self._indices is None:
      self._indices = [[] for _ in range(len(self._angles))]
      for i, d in enumerate(msg.ranges):
        angle = msg.angle_min + i * msg.angle_increment
        for j, center_angle in enumerate(self._angles):
          if _within(angle, center_angle - self._width / 2., center_angle + self._width / 2.):
            self._indices[j].append(i)

    ranges = np.array(msg.ranges)
    for i, idx in enumerate(self._indices):
      # We do not take the minimum range of the cone but the 10-th percentile for robustness.
      self._measurements[i] = np.percentile(ranges[idx], 10)

  @property
  def ready(self):
    return not np.isnan(self._measurements[0])

  @property
  def measurements(self):
    return self._measurements


class GroundtruthPose(object):
  def __init__(self, name='Robot'):
    rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
    self.single_pose = np.array([np.nan, np.nan, np.nan], dtype=np.float32)
    self._pose=np.array([], dtype=np.float32)
    self._name = name

  def callback(self, msg):
    print(msg)
    idx = [i for i, n in enumerate(msg.name) if n[:-1] == self._name]
    print(msg.name)
    if not idx:
      raise ValueError('Specified name "{}" does not exist.'.format(self._name))
    for i in idx:
        self.single_pose[0] = msg.pose[i].position.x
        self.single_pose[1] = msg.pose[i].position.y
        _, _, yaw = euler_from_quaternion([
            msg.pose[i].orientation.x,
            msg.pose[i].orientation.y,
            msg.pose[i].orientation.z,
            msg.pose[i].orientation.w])
        self.single_pose[2] = yaw
        self._pose = np.append(self._pose, self.single_pose)

  @property
  def ready(self):
    return not not len(self._pose)

  @property
  def pose(self):
    return self._pose
  

def run(args):
  rospy.init_node('obstacle_avoidance')
  avoidance_method = globals()[args.mode]

  # Update control every 100 ms.
  rate_limiter = rospy.Rate(100)
  publisher1 = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=5)
  publisher2 = rospy.Publisher('/robot2/cmd_vel', Twist, queue_size=5)
  publisher3 = rospy.Publisher('/robot3/cmd_vel', Twist, queue_size=5)
  publisher4 = rospy.Publisher('/robot4/cmd_vel', Twist, queue_size=5)
  publisher_l=[publisher1, publisher2, publisher3, publisher4]

  laser1 = SimpleLaser(1)
  laser2 = SimpleLaser(2)
  laser3 = SimpleLaser(3)
  laser4 = SimpleLaser(4)
  laser_l=[laser1, laser2, laser3, laser4]
  # Keep track of groundtruth position for plotting purposes.
  groundtruth = GroundtruthPose()
  pose_history = []

  #with open('/tmp/gazebo_exercise.txt', 'w'):
  #  pass

  while not rospy.is_shutdown():
    # Make sure all measurements are ready.
    if not laser1.ready  or not laser2.ready or not laser3.ready or not laser4.ready or not groundtruth.ready:
      rate_limiter.sleep()
      continue
    for r in range(NUM_ROBOTS):
      u, w = avoidance_method(*laser_l[r].measurements)
      vel_msg = Twist()
      vel_msg.linear.x = u
      vel_msg.angular.z = w
      publisher_l[r].publish(vel_msg)
    # Log groundtruth positions in /tmp/gazebo_exercise.txt
    '''
    pose_history.append(groundtruth.pose)
    if len(pose_history) % 10:
      with open('/tmp/gazebo_exercise.txt', 'a') as fp:
        fp.write('\n'.join(','.join(str(v) for v in p) for p in pose_history) + '\n')
        pose_history = []
    rate_limiter.sleep()
    '''


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='Runs obstacle avoidance')
  parser.add_argument('--mode', action='store', default='braitenberg', help='Method.', choices=['braitenberg', 'rule_based'])
  args, unknown = parser.parse_known_args()
  try:
    run(args)
  except rospy.ROSInterruptException:
    pass
