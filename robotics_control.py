
# Robot motion commands:
# http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
from geometry_msgs.msg import Twist
import numpy as np
from easydict import EasyDict

from constant import *

class RobotAbstract():
    def __init__(self, publisher,
                 global_config,
                 config,
                 sensor,
                 positioning):
        self.sensor = sensor
        self.publisher = publisher
        self.type = config.type
        self.name = config.name
        self.positioning = positioning
        self.global_config = global_config
        self.config = config
        self.terminate = False

    def set_instance_dict(self, instance_dict):
        self.instance_dict = instance_dict

    def get_instance_by_name(self, name):
        return self.instance_dict.get(name, None)

    def action(self):

        if self.terminate:
            u, w = 0, 0
            vel_msg = Twist()
            vel_msg.linear.x = u
            vel_msg.angular.z = w
            self.publisher.publish(vel_msg)
            return

        # First process measurements and observations
        groundtruth = self.positioning
        EPSILON = self.config.epsilon

        # Get absolute positioning
        absolute_point_position = np.array([
            groundtruth.pose[X] + EPSILON * np.cos(groundtruth.pose[YAW]),
            groundtruth.pose[Y] + EPSILON * np.sin(groundtruth.pose[YAW])], dtype=np.float32)

        point_position = absolute_point_position
        goal_position = GOAL_POSITION
        pose = groundtruth.pose
        laser_measurements = self.sensor.measurements
        observations = groundtruth.perceived_poses
        # print('other observations', groundtruth.perceived_poses)
        # Pass measurements to controller
        measurements = EasyDict(point_position=point_position,
                                goal_position=goal_position,
                                observations=observations,
                                groundtruth_pose=pose,
                                laser_measurements=laser_measurements)

        u, w = self.controller(measurements=measurements)

        # measurements = self.sensor.measurements
        # u, w = self.controller(*measurements)
        vel_msg = Twist()
        vel_msg.linear.x = u
        vel_msg.angular.z = w
        self.publisher.publish(vel_msg)
        # print('robot', self.name, 'actioned', u, w)

    def stop(self):
        self.terminate = True
        print('Robot', self.name, 'is terminated!')

    def controller(self, *args, **kwargs):
        raise NotImplementedError('No Controller is specified!')

    @property
    def current_position(self):
        return self.positioning.pose

class Police(RobotAbstract):
    def __init__(self, publisher, global_config, config, sensor, positioning):
        RobotAbstract.__init__(self, publisher, global_config, config, sensor, positioning)
        self.captured = set()
        self.current_target = None

    def set_target(self, target_name):
        # Change chasing target
        self.current_target = target_name

    def get_current_target(self):
        return self.current_target

    def add_capture(self, captured):
        print(self.name, 'captures', captured, 'at', self.current_position)
        self.captured.union(captured)

    def controller(self, *args, **kwargs):
        # u, w = braitenberg(*args)
        m = kwargs['measurements']
        # v = get_velocity(m.point_position,
        #                  m.goal_position,
        #                  m.observations,
        #                  max_speed=self.config.max_speed)
        v = self.get_potential_field(m.point_position, m.observations)
        u, w = feedback_linearized(m.groundtruth_pose, v, epsilon=self.config.epsilon)
        # print('robot', self.name,
        #       'current pos', m.groundtruth_pose,
        #       'control signals', u, w)
        return u, w

    def get_potential_field(self, point_position, observations):
        # Police get potential field
        # Baddies are targets

        baddy_names = [robot.name for robot in self.global_config.robots if (robot.type == 'baddy' and robot.name not in self.captured)]
        police_names = [robot.name for robot in self.global_config.robots if robot.type == 'police']
        baddies = {}
        police = {}
        for obj_name, obj in observations.items():
            if obj_name in baddy_names:
                baddies[obj_name] = obj
            elif obj_name in police_names:
                police[obj_name] = obj

        combined_v = np.zeros(2, dtype=np.float32)

        # avoid hitting other police
        for police_name, police_data in police.items():
            combined_v += get_velocity_to_avoid_obstacles(point_position,
                                                            [police_data.data[:2]],
                                                            [ROBOT_RADIUS + ROBOT_RADIUS],
                                                            max_speed=self.config.max_speed,
                                                            scale_factor=10)

        # avoid hitting other captured baddies
        for baddy_name, baddy_data in baddies.items():
            if self.get_instance_by_name(baddy_name).free:
                continue
            combined_v += get_velocity_to_avoid_obstacles(point_position,
                                                            [baddy_data.data[:2]],
                                                            [ROBOT_RADIUS + ROBOT_RADIUS],
                                                            max_speed=self.config.max_speed,
                                                            scale_factor=10)

        # If has a target baddy
        if self.current_target is not None:
            goal_position = baddies[self.current_target].data[:2]
            v_chase_baddy = get_velocity_to_reach_goal(point_position, goal_position,
                                            max_speed=self.config.max_speed)
            combined_v += v_chase_baddy

        # Avoid hitting walls
        for obstacle in self.global_config.obstacles:
            if obstacle.params.type == 'square_wall':
                v_avoid = get_velocity_to_avoid_walls(point_position, obstacle, max_speed=self.config.max_speed)
                combined_v += v_avoid

        # v_avoid = get_velocity_to_avoid_obstacles(point_position,
        #                                           [obs.data.position for obs in observations.values() if
        #                                                            obs.type == 'cylinder'],
        #                                           [ROBOT_RADIUS + obs.data.radius for obs in observations.values() if
        #                                            obs.type == 'cylinder'],
        #                                           max_speed=self.config.max_speed)
        # combined_v += v_avoid

        combined_v = cap(combined_v, max_speed=self.config.max_speed)
        return combined_v

class Baddy(RobotAbstract):
    def __init__(self, publisher, global_config, config, sensor, positioning):
        RobotAbstract.__init__(self, publisher, global_config, config, sensor, positioning)
        self.free = True
        self.capture_by = set()

    def get_captured_by(self, capture_by):
        self.capture_by.union(capture_by)
        self.free = False
        self.stop()

    def controller(self, *args, **kwargs):
        m = kwargs['measurements']
        # laser_measurements = m.laser_measurements
        # u, w = braitenberg(*laser_measurements)
        v = self.get_potential_field(m.point_position, m.observations)
        u, w = feedback_linearized(m.groundtruth_pose, v, epsilon=self.config.epsilon)
        return u, w


    def get_potential_field(self, point_position, observations):
        # Baddies get potential field
        # Police are obstacles

        baddy_names = [robot.name for robot in self.global_config.robots if robot.type == 'baddy']
        police_names = [robot.name for robot in self.global_config.robots if robot.type == 'police']
        baddies = {}
        police = {}
        for obj_name, obj in observations.items():
            if obj_name in baddy_names:
                baddies[obj_name] = obj
            elif obj_name in police_names:
                police[obj_name] = obj

        combined_v = np.zeros(2, dtype=np.float32)

        # avoid hitting other baddies
        for baddy_name, baddy_data in baddies.items():
            combined_v += get_velocity_to_avoid_obstacles(point_position,
                                                          [baddy_data.data[:2]],
                                                          [ROBOT_RADIUS + ROBOT_RADIUS],
                                                          max_speed=self.config.max_speed,
                                                          scale_factor=10)

        # escape from the police
        for police_name, police_data in police.items():
            v_escape = get_velocity_to_avoid_obstacles(point_position,
                                                      [police_data.data[:2]],
                                                      [ROBOT_RADIUS],
                                            max_speed=self.config.max_speed,
                                                      scale_factor=5)
            combined_v += v_escape

        # Avoid hitting walls
        for obstacle in self.global_config.obstacles:
            if obstacle.params.type == 'square_wall':
                v_avoid = get_velocity_to_avoid_walls(point_position, obstacle, max_speed=self.config.max_speed)
                combined_v += v_avoid


        v_avoid = get_velocity_to_avoid_obstacles(point_position,
                                                   [obs.data.position for obs in observations.values() if
                                                                    obs.type == 'cylinder'],
                                                   [ROBOT_RADIUS + obs.data.radius for obs in observations.values() if
                                                    obs.type == 'cylinder'],
                                                   max_speed=self.config.max_speed)
        combined_v += v_avoid
        combined_v = cap(combined_v, max_speed=self.config.max_speed)
        return combined_v


########## Control functions ##########

def avoid(distance):
  return max(0,0.5-np.exp(0.05-distance))

def braitenberg(front, front_left, front_right, left, right):
    u = 0.  # [m/s]
    w = 0.  # [rad/s] going counter-clockwise.
    # front=1 if math.isinf(front) else  front
    # front_left=1 if math.isinf(front_left) else front_left
    # front_right=1 if math.isinf(front_right) else  front_right
    # left=1 if math.isinf(left) else  left
    # right=1 if math.isinf(right) else  right
    # sys.stdout.write(str(front)+" "+str(left)+"\n")
    # sys.stdout.flush()
    vl = (avoid(right) + avoid(front_right)) / 2
    vr = (avoid(left) + avoid(front_left)) / 2

    u = avoid(front)  # =(vr+vl)/2
    w = 0.5 * (vr - vl) + 0.3 * np.exp(-10 * u) * np.sign(front_left - front_right)
    # MISSING: Implement a braitenberg controller that takes the range
    # measurements given in argument to steer the robot
    return u, w


def get_velocity(point_position, goal_position, observations, max_speed):
    v_goal = get_velocity_to_reach_goal(point_position, goal_position,
                                        max_speed=max_speed)
    v_avoid = get_velocity_to_avoid_obstacles(point_position, [obs.data.position for obs in observations.values() if obs.type == 'cylinder'],
                                                              [ROBOT_RADIUS + obs.data.radius for obs in observations.values() if obs.type == 'cylinder'],
                                              max_speed=max_speed)
    v = v_goal + v_avoid
    return cap(v, max_speed=max_speed)

def get_distance(x, y):
  return np.sqrt(np.sum((x - y) ** 2))

def get_unit_vector(x):
  return x / np.sqrt(np.sum(x**2))

def get_velocity_to_reach_goal(position, goal_position, max_speed):
  v = np.zeros(2, dtype=np.float32)
  # MISSING: Compute the velocity field needed to reach goal_position
  # assuming that there are no obstacles.

  # get the unit vector pointing to the goal
  current_distance = get_distance(position, goal_position)
  unit_direction = (goal_position - position) / current_distance
  amplitude = max_speed
  # give the vector the MAX_SPEED amplitude
  v = unit_direction * amplitude

  return v


def get_velocity_to_avoid_obstacles(position,
                                    obstacle_positions,
                                    obstacle_radii,
                                    max_speed,
                                    scale_factor=1):
  v = np.zeros(2, dtype=np.float32)
  # MISSING: Compute the velocity field needed to avoid the obstacles
  # In the worst case there might a large force pushing towards the
  # obstacles (consider what is the largest force resulting from the
  # get_velocity_to_reach_goal function). Make sure to not create
  # speeds that are larger than max_speed for each obstacle. Both obstacle_positions
  # and obstacle_radii are lists.

  # The higher, the quicker the velocity field decays
  # scale_factor = 1

  for i in range(len(obstacle_positions)):
    # Get unit direction to the obstacle center
    obstacle_position = obstacle_positions[i]
    obstacle_radius = obstacle_radii[i]
    to_obs_distance = get_distance(position, obstacle_position)
    unit_direction = (position - obstacle_position) / to_obs_distance
    # Compute the decay factor in the range of [0, 1]
    decay_factor = np.exp(-np.abs((to_obs_distance - obstacle_radius - SECURITY_DISTANCE)) * scale_factor)

    # Assign amplitude
    if to_obs_distance <= obstacle_radius + SECURITY_DISTANCE:
      amplitude = max_speed
    else:
      amplitude = decay_factor * max_speed
    # add to v. There might be multiple obstacles
    v += unit_direction * amplitude

  return v

def get_velocity_to_avoid_walls(position, wall_config, max_speed,
                                scale_factor=0.05):
    # This function returns velocity to avoid hitting walls
    wall_data = wall_config.params
    v = np.zeros(2, dtype=np.float32)
    if wall_data.type == 'square_wall':
        def compute_velocity(distance, direction):
            # Compute the decay factor in the range of [0, 1]
            decay_factor = np.exp(-distance * scale_factor)
            # Assign amplitude
            amplitude = decay_factor * max_speed
            return direction * amplitude

        left_x = wall_data.position[X]
        right_x = wall_data.position[X] + wall_data.dx
        bottom_y = wall_data.position[Y]
        top_y = wall_data.position[Y] + wall_data.dy

        to_wall_distance = abs(position[X] - left_x)
        sign = (position[X]-left_x) / to_wall_distance
        to_wall_direction = sign * np.array([1, 0], dtype=np.float32) * max_speed
        v += compute_velocity(to_wall_distance, to_wall_direction)
        # print('left adding', compute_velocity(to_wall_distance, to_wall_direction))

        to_wall_distance = abs(position[X] - right_x)
        sign = (position[X] - right_x) / to_wall_distance
        to_wall_direction = sign * np.array([1, 0], dtype=np.float32) * max_speed
        v += compute_velocity(to_wall_distance, to_wall_direction)
        # print('right adding', compute_velocity(to_wall_distance, to_wall_direction))

        to_wall_distance = abs(position[Y] - bottom_y)
        sign = (position[Y] - bottom_y) / to_wall_distance
        to_wall_direction = sign * np.array([0, 1], dtype=np.float32) * max_speed
        v += compute_velocity(to_wall_distance, to_wall_direction)
        # print('bottom adding', compute_velocity(to_wall_distance, to_wall_direction))

        to_wall_distance = abs(position[Y] - top_y)
        sign = (position[Y] - top_y) / to_wall_distance
        to_wall_direction = sign * np.array([0, 1], dtype=np.float32) * max_speed
        v += compute_velocity(to_wall_distance, to_wall_direction)
        # print('up adding', compute_velocity(to_wall_distance, to_wall_direction))

    # print(position, v)

    return cap(v, max_speed=max_speed)


def normalize(v):
  n = np.linalg.norm(v)
  if n < 1e-2:
    return np.zeros_like(v)
  return v / n


def cap(v, max_speed):
  n = np.linalg.norm(v)
  if n > max_speed:
    return v / n * max_speed
  return v

def feedback_linearized(pose, velocity, epsilon):
  u = 0.  # [m/s]
  w = 0.  # [rad/s] going counter-clockwise.

  # MISSING: Implement feedback-linearization to follow the velocity
  # vector given as argument. Epsilon corresponds to the distance of
  # linearized point in front of the robot.
  u = velocity[X] * np.cos(pose[YAW]) + velocity[Y] * np.sin(pose[YAW])
  w = 1 / epsilon * (-velocity[X] * np.sin(pose[YAW]) + velocity[Y] * np.cos(pose[YAW]))
  return u, w

