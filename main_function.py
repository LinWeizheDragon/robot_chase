#!/usr/bin/env python

'''
Main functions
'''
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
import json
import pickle
import os
import argparse
import numpy as np
import rospy
import sys
from easydict import EasyDict
from visualization_msgs.msg import Marker
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
# Robot motion commands:
# http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
from geometry_msgs.msg import Twist, PoseStamped
# Laser scan message:
# http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
# For groundtruth information.
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion

from laser_scanner import SimpleLaser
from get_random_init import initial_position
from robotics_control import Police, Baddy, get_distance, get_unit_vector
from constant import *
from global_positioning import GroundtruthPose
from config import get_config_by_name
from std_srvs.srv import Empty
from metrics import MetricsManager

reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
start_simulation = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
pause_simulation = rospy.ServiceProxy('/gazebo/pause_physics', Empty)


def run(config, run_id=0):
    '''
    Run experiment
    :param config: experiment global config
    :param run_id: current run id
    :return: log_data from metrics manager. See metrics.MetricsManager.get_log_data()
    '''
    rospy.init_node('obstacle_avoidance')

    # Update control every 100 ms.
    rate_limiter = rospy.Rate(50)

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

    # Create metrics logger
    metrics_manager = MetricsManager(configs)
    metrics_manager.set_instance_dict(instance_dict)

    # Start simulation automatically
    reset_simulation()
    state_mgses, config = initial_position(config, run_id)
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        for msg in state_mgses:
            resp = set_state( msg )
    except rospy.ServiceException as e:
        print ("Service call failed: %s" % e)
    print('simulation reset!')
    start_simulation()

    def save_experiments():
        '''
        Save experiment data to files
        :return:
        '''
        print('start saving this experiment...')
        experiment_data = EasyDict(
            robot_data=[],
            metrics_data=metrics_manager.get_log_data(),
            config=config,
        )
        for robot in robot_intances:
            experiment_data.robot_data.append(robot.history)

        dir_path = os.path.join(
            '../catkin_ws/src/robot_chase/experiments', config.experiment_name)
        if not os.path.exists(dir_path):
            os.makedirs(dir_path)
        save_path = os.path.join(dir_path, 'run_{}.pkl'.format(run_id))
        with open(save_path, 'wb') as f:
            pickle.dump(experiment_data, f)
            print('experiment saved to {}'.format(save_path))

    def lprint(*args):
        '''
        Print args while saving to metrics manager
        :param args:
        :return:
        '''
        print(*args)
        metrics_manager.add_log(*args)


    frame_id = 0
    import time
    start_time = rospy.get_rostime()
    while not rospy.is_shutdown():
        current_time = rospy.get_rostime()
        # Make sure all measurements are ready.
        if not all([robot.sensor.ready for robot in robot_intances]):
            rate_limiter.sleep()
            continue
        if not all([robot.positioning.ready for robot in robot_intances]):
            rate_limiter.sleep()
            continue

        if (current_time - start_time).to_sec() > config.timeout:
            # timeout
            lprint('TIMEOUT!!!!! simulation finished!')

            # Pause simulation
            pause_simulation()

            # Log necessary results
            if config.mode == 'test':
                save_experiments()
            return metrics_manager.get_log_data()


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
                    lprint(police.name, 'captured', baddy.name, 'at', police.current_position)
                if baddy.free:
                    if dist < nearest_baddy[0]:
                        # This baddy is more close
                        nearest_baddy = (dist, baddy)

            if nearest_baddy[1] is None:
                # No free baddies
                lprint('no free baddies running, stop all agents')
                police.stop()
                lprint('simulation finished!')

                # finally update the states
                metrics_manager.update(frame_id)

                # Pause simulation
                pause_simulation()

                # Log necessary results
                if config.mode == 'test':
                    save_experiments()
                return metrics_manager.get_log_data()
            else:
                # simple strategy to chase the nearest baddy
                if police.current_target == nearest_baddy[1].name:
                    pass
                else:
                    lprint(police.name, 'changed its target to', nearest_baddy[1].name)
                    police.set_target(nearest_baddy[1].name)

        # update metrics every iteration
        metrics_manager.update(frame_id)
        frame_id += 1
        # time_elapsed = time.time() - current_time
        # print('time passed:', time_elapsed)
        # current_time = time.time()
        # rate_limiter.sleep()
        # time_elapsed = time.time() - current_time
        # print('sleep for:', time_elapsed)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs obstacle avoidance')
    parser.add_argument('--mode', action='store', default='debug',
                        help='debug for single run; test for running a few experiments',
                        choices=['debug', 'test'])
    parser.add_argument('--config', action='store', type=str, required=True,
                        help='config name.')
    parser.add_argument('--experiment_name', action='store',
                        type=str, default='debug_experiment',
                        help='path to save experiments.')
    parser.add_argument('--num_test', action='store', type=str,
                        default='1',
                        help='num of test runs.')
    parser.add_argument('--max_speed', type=str,
                        help='list of max_speed that will be assigned to each robot (in order)')
    parser.add_argument('--strategy', action='store', default='naive',
                        help='chase strategy',
                        choices=['naive', 'estimation'])
    parser.add_argument('--velocity_component', type=str,
                        help='list of velocity component.')
    parser.add_argument('--visibility', action='store', type=str,
                        default='',
                        help='visibility of robots (in order)')
    parser.add_argument('--timeout', action='store', type=str,
                        default='90',
                        help='timeout in seconds')

    args, unknown = parser.parse_known_args()

    print(args)
    print('CWD:', os.getcwd())
    print("Python Version {}".format(str(sys.version).replace('\n', '')))

    configs = get_config_by_name(config_name=args.config)

    configs = EasyDict(configs)
    configs.num_robots = len(configs.robots)
    configs.timeout = int(args.timeout)

    # Evaluate args, override params
    mode = args.mode
    if mode == 'debug':
        num_test = 1
    else:
        num_test = int(args.num_test)
    experiment_name = args.experiment_name
    configs.experiment_name = experiment_name
    configs.strategy = args.strategy
    configs.mode = mode

    if args.visibility != "none":
        visibility_list = args.visibility.split(',')
        assert len(visibility_list) == configs.num_robots
        for i, robot_config in enumerate(configs.robots):
            robot_config.visibility = float(visibility_list[i])

    if args.max_speed != "none":
        max_speed_list = args.max_speed.split(',')
        assert len(max_speed_list) == configs.num_robots
        for i, robot_config in enumerate(configs.robots):
            robot_config.max_speed = float(max_speed_list[i])

    if args.velocity_component != "none":
        velocity_component_list = args.velocity_component.split(',')
        assert len(velocity_component_list) == len(configs.velocity_component)
        for i, component_key in enumerate(configs.velocity_component.keys()):
            configs.velocity_component[component_key] = float(velocity_component_list[i])

    print(configs)

    try:
        run_id = 0
        all_success = []
        while run_id<num_test:
            try:
                print('start running Experiment {} : {} / {}'.format(experiment_name, run_id+1, num_test))
                result = run(configs, run_id)
                run_id += 1
                all_success.append(result.all_success)
            except rospy.ROSTimeMovedBackwardsException:
                print('catch ROSTimeMovedBackwardsException, ignore and restart!')
                pass

        # Processing and output result metrics
        all_success_rate = np.mean(np.array(all_success))
        print('all experiments finished.')
        print('all_success_rate', all_success_rate)

        dir_path = os.path.join(
            '../catkin_ws/src/robot_chase/experiments', configs.experiment_name)
        if not os.path.exists(dir_path):
            os.makedirs(dir_path)
        save_path = os.path.join(dir_path, 'metrics.json'.format(run_id))
        with open(save_path, 'w') as f:
            json.dump({
                'all_success': all_success,
                'all_success_rate': all_success_rate,
                'num_test': num_test
            }, f, indent=4)
            print('all experiments saved to {}'.format(save_path))

    except rospy.ROSInterruptException:
        pass
