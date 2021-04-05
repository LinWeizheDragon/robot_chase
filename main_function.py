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
from visualization_msgs.msg import Marker
# Robot motion commands:
# http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
from geometry_msgs.msg import Twist, PoseStamped
# Laser scan message:
# http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
# For groundtruth information.
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion

from laser_scanner import SimpleLaser
from robotics_control import Police, Baddy, get_distance, get_unit_vector
from constant import *
from global_positioning import GroundtruthPose


def run(config):
    rospy.init_node('obstacle_avoidance')
    # avoidance_method = globals()[args.mode]

    # Update control every 100 ms.
    rate_limiter = rospy.Rate(100)

    robot_intances = []
    police_intances = []
    baddy_instances = []
    instance_dict = {}
    for robot_index, robot in enumerate(config.robots):

        print(robot)
        velocity_publisher = rospy.Publisher('/{}/cmd_vel'.format(robot.name), Twist, queue_size=5)
        pose_publisher = rospy.Publisher('/{}/pose'.format(robot.name), PoseStamped, queue_size=5)
        marker_publisher = rospy.Publisher('/{}/marker'.format(robot.name), Marker, queue_size=5)
        goal_publisher = rospy.Publisher('/{}/goal'.format(robot.name), Marker, queue_size=5)
        if robot.type == 'police':
            RobotClass = Police
            PositioningClass = GroundtruthPose
            SensorClass = SimpleLaser
        else:
            RobotClass = Baddy
            PositioningClass = GroundtruthPose
            SensorClass = SimpleLaser

        sensor = SensorClass(robot)
        positioning = PositioningClass(config, robot)
        robot_instance = RobotClass(velocity_publisher, pose_publisher, marker_publisher, goal_publisher, config, robot, sensor, positioning)
        robot_intances.append(robot_instance)
        instance_dict[robot.name] = robot_instance
        if robot.type == 'police':
            police_intances.append(robot_instance)
        else:
            baddy_instances.append(robot_instance)

    # Pass {robot_name: robot_instance} to each instance for future use
    for robot in robot_intances:
        robot.set_instance_dict(instance_dict)

    while not rospy.is_shutdown():
        frame_id = 0
        # Make sure all measurements are ready.
        if not all([robot.sensor.ready for robot in robot_intances]):
            rate_limiter.sleep()
            continue
        if not all([robot.positioning.ready for robot in robot_intances]):
            rate_limiter.sleep()
            continue

        for i, robot in enumerate(robot_intances):
            robot.action(frame_id)

        # Conduct strategy
        for i, police in enumerate(police_intances):
            if police.terminate:
                continue
            nearest_baddy = (np.inf, None)
            for j, baddy in enumerate(baddy_instances):
                if not baddy.free:
                    # this has been captured
                    continue

                dist = get_distance(police.current_position, baddy.current_position)
                if dist<CAPTURE_DISTANCE:
                    # This police captures the baddy
                    police.add_capture(baddy.name)
                    baddy.get_captured_by(police.name)

                if baddy.free:
                    if dist < nearest_baddy[0]:
                        # This baddy is more close
                        nearest_baddy = (dist, baddy)

            if nearest_baddy[1] is None:
                # No free baddies
                print('no free baddies running, stop the agent')
                police.stop()
            else:
                # simple strategy to chase the nearest baddy
                if police.current_target == nearest_baddy[1].name:
                    pass
                else:
                    print(police.name, 'changed its target to', nearest_baddy[1].name)
                    police.set_target(nearest_baddy[1].name)
        frame_id += 1



if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs obstacle avoidance')
    parser.add_argument('--mode', action='store', default='braitenberg', help='Method.',
                        choices=['braitenberg', 'rule_based'])
    args, unknown = parser.parse_known_args()

    print(args)
    print("Python Version {}".format(str(sys.version).replace('\n', '')))

    configs = {
        'dt': 0.05,
        'robots': [
            {
                'name': 'robot1',
                'type': 'police',
                'epsilon': 0.2,
                'max_speed': 0.6,
            },
            {
                'name': 'robot2',
                'type': 'police',
                'epsilon': 0.2,
                'max_speed': 0.6,
            },
            {
                'name': 'robot3',
                'type': 'baddy',
                'epsilon': 0.2,
                'max_speed': 0.9,
            },
            {
                'name': 'robot4',
                'type': 'baddy',
                'epsilon': 0.2,
                'max_speed': 0.9,
            },
        ],
        'obstacles': [
            {
                'name': 'cylinder1',
                'params': {
                    'type': 'cylinder1',
                    'position': np.array([3, 2], dtype=np.float32),
                    'radius': 1.5,
                }
            },
            {
                'name': 'cylinder2',
                'params': {
                    'type': 'cylinder1',
                    'position': np.array([-1, -5.5], dtype=np.float32),
                    'radius': 1.3,
                }
            },
            {
                'name': 'cylinder3',
                'params': {
                    'type': 'cylinder1',
                    'position': np.array([-4, 3.5], dtype=np.float32),
                    'radius': 1.2,
                }
            },
            {
                'name': 'walls',
                'params': {
                    'type': 'square_wall',
                    'position': np.array([-8.64, -8.64], dtype=np.float32),
                    'dx': 8.64*2,
                    'dy': 8.64*2,
                }
            }
        ]
    }

    configs = EasyDict(configs)
    configs.num_robots = len(configs.robots)

    print(configs)

    try:
        run(configs)
    except rospy.ROSInterruptException:
        pass
