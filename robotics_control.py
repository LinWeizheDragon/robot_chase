# Robot motion commands:
# http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
from geometry_msgs.msg import Twist, PoseStamped
from visualization_msgs.msg import Marker
import numpy as np
from easydict import EasyDict
import rospy
from constant import *
from tf.transformations import quaternion_from_euler


class RobotAbstract():
    def __init__(self, velocity_publisher, pose_publisher, marker_publisher, goal_publisher,
                 global_config,
                 config,
                 sensor,
                 positioning):
        self.sensor = sensor
        self.publisher = velocity_publisher
        self.pose_publisher = pose_publisher
        self.marker_publisher = marker_publisher
        self.goal_publisher = goal_publisher
        self.type = config.type
        self.name = config.name
        self.positioning = positioning
        self.global_config = global_config
        self.config = config
        self.frame_id = 0
        self.terminate = False
        #self.pose_estimator = PoseEstimator(global_config=global_config,
                                            #config=config)

        self.history = EasyDict(
            config=config,
            measurements=[],
            action=[],
            pose_estimation=[],
        )

    def set_instance_dict(self, instance_dict):
        self.instance_dict = instance_dict

    def get_instance_by_name(self, name):
        return self.instance_dict.get(name, None)

    def action(self, frame_id):
        if self.terminate:
            u, w, v = 0, 0, 0
            vel_msg = Twist()
            vel_msg.linear.x = u
            vel_msg.angular.z = w
            self.frame_id = frame_id
            self.publisher.publish(vel_msg)
            return

        # First process measurements and observations
        groundtruth = self.positioning.pose(self.name)
        EPSILON = self.config.epsilon

        # Get absolute positioning
        absolute_point_position = np.array([
            groundtruth[X] + EPSILON * np.cos(groundtruth[YAW]),
            groundtruth[Y] + EPSILON * np.sin(groundtruth[YAW])], dtype=np.float32)

        point_position = absolute_point_position
        goal_position = GOAL_POSITION
        pose = groundtruth
        laser_measurements = self.sensor.measurements
        observations = self.positioning.perceived_poses
        # print('other observations', groundtruth.perceived_poses)

        # Process observations using PoseEsimator
        #self.pose_estimator.process_observations(observations)

        # Pass measurements to controller
        measurements = EasyDict(point_position=point_position,
                                goal_position=goal_position,
                                observations=observations,
                                groundtruth_pose=pose,
                                laser_measurements=laser_measurements)

        u, w, v = self.controller(measurements=measurements)
        # measurements = self.sensor.measurements
        # u, w = self.controller(*measurements)
        vel_msg = Twist()
        vel_msg.linear.x = u
        vel_msg.angular.z = w
        self.publisher.publish(vel_msg)

        # log historical actions and measurements
        self.history.action.append(EasyDict(u=u, w=w, v=v))
        self.history.measurements.append(measurements)
        #self.history.pose_estimation.append(
            #self.pose_estimator.distribution_dict)

        pose_msg = generate_pose_msg(self.pose_publisher, v, point_position, frame_id)
        generate_marker(self.marker_publisher, self.name, v, pose_msg, frame_id)

    def stop(self):
        self.terminate = True
        print('Robot', self.name, 'is terminated!')

    def controller(self, *args, **kwargs):
        raise NotImplementedError('No Controller is specified!')

    @property
    def current_position(self):
        return self.positioning.pose(self.name)


class Police(RobotAbstract):
    def __init__(self, publisher, pose_publisher, marker_publisher, goal_publisher, global_config, config, sensor,
                 positioning):
        RobotAbstract.__init__(self, publisher, pose_publisher, marker_publisher, goal_publisher, global_config, config,
                               sensor, positioning)
        self.captured = set()
        self.current_target = None

    def set_target(self, target_name):
        # Change chasing target
        self.current_target = target_name

    def get_current_target(self):
        return self.current_target

    def add_capture(self, captured):
        print(self.name, 'captures', captured, 'at', self.current_position)
        self.captured.add(captured)
        # print(self.name, self.captured)

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
        # if self.name == 'robot1':
        #    print(self.name, v,u,w,np.cos(m.groundtruth_pose[YAW]), np.sin(m.groundtruth_pose[YAW]), m.groundtruth_pose[:2])
        return u, w, v

    def get_potential_field(self, point_position, observations):
        # Police get potential field
        # Baddies are targets
        # print('police estimation')
        # print(
        #    self.name,
        #    self.pose_estimator.distribution_dict
        # )
        baddy_names = [robot.name for robot in self.global_config.robots if
                       (robot.type == 'baddy' and robot.name not in self.captured)]
        police_names = [robot.name for robot in self.global_config.robots if robot.type == 'police']
        baddies = {}
        police = {}
        for obj_name, obj in observations.items():
            if obj_name in baddy_names:
                baddies[obj_name] = obj
            elif obj_name in police_names:
                police[obj_name] = obj

        v_dict = EasyDict()

        # avoid hitting other police
        v_dict.avoid_hitting_police = np.zeros(2, dtype=np.float32)
        for police_name, police_data in police.items():
            if police_name == self.name:
                continue
            v_dict.avoid_hitting_police += get_velocity_to_avoid_obstacles(point_position,
                                                                           [police_data.data.pose[:2]],
                                                                           [
                                                                               ROBOT_RADIUS + ROBOT_RADIUS + SECURITY_DISTANCE.companion],
                                                                           max_speed=self.config.max_speed,
                                                                           scale_factor=10,
                                                                           prune_distance=0.5)

        # avoid hitting other captured baddies
        v_dict.avoid_hitting_captured_baddies = np.zeros(2, dtype=np.float32)
        for baddy_name, baddy_data in baddies.items():
            if self.get_instance_by_name(baddy_name).free:
                continue
            v_dict.avoid_hitting_captured_baddies += get_velocity_to_avoid_obstacles(point_position,
                                                                                     [baddy_data.data.pose[:2]],
                                                                                     [
                                                                                         ROBOT_RADIUS + ROBOT_RADIUS + SECURITY_DISTANCE.obstacle],
                                                                                     max_speed=self.config.max_speed,
                                                                                     scale_factor=10,
                                                                                     prune_distance=0.5)

        # If has a target baddy
        if self.current_target is not None:

            if self.global_config.strategy == 'naive':
                goal_position = baddies[self.current_target].data.pose[:2]
            elif self.global_config.strategy == 'estimation':
                goal_pose = self.pose_estimator.get_estimated_distribution(self.current_target,
                                                                           observations[self.current_target].data,
                                                                           step=30).mean
                goal_position = goal_pose[:2]

            generate_marker(self.goal_publisher, self.name, 0, goal_position, self.frame_id, goal=True)

            v_chase_baddy = get_velocity_to_reach_goal(point_position,
                                                       goal_position,
                                                       max_speed=self.config.max_speed)
            v_dict.chase_baddy = v_chase_baddy

        # Avoid hitting walls
        v_dict.avoid_hitting_walls = np.zeros(2, dtype=np.float32)
        for obstacle in self.global_config.obstacles:
            if obstacle.params.type == 'square_wall':
                v_avoid = get_velocity_to_avoid_walls(point_position, obstacle, max_speed=self.config.max_speed)
                v_dict.avoid_hitting_walls += v_avoid

        # v_avoid = get_velocity_to_avoid_obstacles(point_position,
        #                                           [obs.data.position for obs in observations.values() if
        #                                            obs.type == 'cylinder'],
        #                                           [ROBOT_RADIUS + obs.data.radius + SECURITY_DISTANCE.obstacle for obs in observations.values() if
        #                                            obs.type == 'cylinder'],
        #                                           max_speed=self.config.max_speed)
        v_avoid = get_velocity_to_avoid_obstacles(point_position,
                                                  [obs.params.position for obs in self.global_config.obstacles if
                                                   obs.params.type == 'cylinder'],
                                                  [ROBOT_RADIUS + SECURITY_DISTANCE.obstacle + obs.params.radius for obs
                                                   in self.global_config.obstacles if obs.params.type == 'cylinder'],
                                                  max_speed=self.config.max_speed,
                                                  prune_distance=0.5)
        v_dict.avoid_hitting_obstacles = v_avoid

        combined_v = np.zeros(2, dtype=np.float32)
        # print('====', self.name, ' start====')
        for v_description, v_value in v_dict.items():
            #    print(v_description, v_value, get_magnitude(v_value))
            combined_v += v_value * self.global_config.velocity_component[v_description]
        # print('combined:', combined_v, get_magnitude(combined_v))
        # print('====', self.name, ' end====')

        combined_v = cap(combined_v, max_speed=self.config.max_speed)
        return combined_v


class Baddy(RobotAbstract):
    def __init__(self, publisher, pose_publisher, maker_publisher, goal_publisher, global_config, config, sensor,
                 positioning):
        RobotAbstract.__init__(self, publisher, pose_publisher, maker_publisher, goal_publisher, global_config, config,
                               sensor, positioning)
        self.free = True
        self.capture_by = set()

    def get_captured_by(self, capture_by):
        self.capture_by.add(capture_by)
        # print(self.name, self.capture_by)
        self.free = False
        self.stop()

    def controller(self, *args, **kwargs):
        m = kwargs['measurements']
        # laser_measurements = m.laser_measurements
        # u, w = braitenberg(*laser_measurements)
        v = self.get_potential_field(m.point_position, m.observations)
        if np.linalg.norm(v) < 1e-2:
            v = add_noise(v)
        u, w = feedback_linearized(m.groundtruth_pose, v, epsilon=self.config.epsilon)

        return u, w, v

    def get_potential_field(self, point_position, observations):
        # Baddies get potential field
        # Police are obstacles
        # print('====', self.name, ' start====')
        baddy_names = [robot.name for robot in self.global_config.robots if robot.type == 'baddy']
        police_names = [robot.name for robot in self.global_config.robots if robot.type == 'police']
        baddies = {}
        police = {}
        for obj_name, obj in observations.items():
            if obj_name in baddy_names:
                baddies[obj_name] = obj
            elif obj_name in police_names:
                police[obj_name] = obj

        v_dict = EasyDict()

        # avoid hitting other baddies
        v_dict.avoid_hitting_baddies = np.zeros(2, dtype=np.float32)
        for baddy_name, baddy_data in baddies.items():
            if baddy_name == self.name:
                continue
            v_dict.avoid_hitting_baddies += get_velocity_to_avoid_obstacles(point_position,
                                                                            [baddy_data.data.pose[:2]],
                                                                            [
                                                                                ROBOT_RADIUS + ROBOT_RADIUS + SECURITY_DISTANCE.companion],
                                                                            max_speed=self.config.max_speed,
                                                                            scale_factor=5,
                                                                            prune_distance=1)
        # if self.frame_id%1000 ==0:
        #    print(self.name, "other baddy: ", combined_v)
        # escape from the police
        v_dict.escape_from_police = np.zeros(2, dtype=np.float32)
        for police_name, police_data in police.items():
            v_escape = get_velocity_to_avoid_obstacles(point_position,
                                                       [police_data.data.pose[:2]],
                                                       [ROBOT_RADIUS + SECURITY_DISTANCE.competitor],
                                                       max_speed=self.config.max_speed,
                                                       scale_factor=1,
                                                       prune_distance=np.inf)
            v_dict.escape_from_police += v_escape

        # Avoid hitting walls
        v_dict.avoid_hitting_walls = np.zeros(2, dtype=np.float32)
        for obstacle in self.global_config.obstacles:
            if obstacle.params.type == 'square_wall':
                v_avoid = get_velocity_to_avoid_walls(point_position, obstacle, max_speed=self.config.max_speed)
                v_dict.avoid_hitting_walls += v_avoid
        # if self.name == 'robot3' and self.frame_id%1000000 ==0:
        # print(v_dict.avoid_hitting_walls,v_dict.escape_from_police)

        v_avoid = get_velocity_to_avoid_obstacles(point_position,
                                                  [obs.params.position for obs in self.global_config.obstacles if
                                                   obs.params.type == 'cylinder'],
                                                  [ROBOT_RADIUS + SECURITY_DISTANCE.obstacle + obs.params.radius for
                                                   obs in self.global_config.obstacles if
                                                   obs.params.type == 'cylinder'],
                                                  max_speed=self.config.max_speed,
                                                  prune_distance=0.5)
        v_dict.avoid_hitting_obstacles = v_avoid


        v_avoid_corner = get_velocity_to_avoid_obstacles(point_position,
                                                         self.global_config.corners,
                                                         [0.5, 0.5, 0.5, 0.5],
                                                         scale_factor=0.5,
                                                         max_speed=self.config.max_speed,
                                                         prune_distance=7)
        v_dict.avoid_corners = v_avoid_corner

        combined_v = np.zeros(2, dtype=np.float32)

        for v_description, v_value in v_dict.items():
            # if self.frame_id%1000000 ==0:
            # Prune the inidividual v component before adding
            v_value = cap(v_value, self.config.max_speed)
            # print(self.name, v_description, v_value, get_magnitude(v_value))
            combined_v += v_value * self.global_config.velocity_component[v_description]
        # print('combined:', combined_v, get_magnitude(combined_v))
        # print('====', self.name, ' end====')

        combined_v = cap(combined_v, max_speed=self.config.max_speed)
        # anyways try to escape
        if get_magnitude(combined_v) < (0.5 * self.config.max_speed):
            if get_magnitude(v_dict.escape_from_police) > (0.8 * self.config.max_speed):
                combined_v = self.config.max_speed * combined_v / get_magnitude(combined_v)

        return combined_v


NOT_READY = 0
ACCURATE = 1
ESTIMATED = 2


class PoseEstimator():
    '''
    This class is a unit that estimates the position of other agents
    '''

    def __init__(self, global_config, config):
        self.name = config.name  # self name
        self.global_config = global_config
        self.config = config

        self.distribution_dict = {}
        for robot_config in self.global_config.robots:
            self.distribution_dict[robot_config.name] = {
                'mean': np.zeros(3, dtype=np.float32),
                'variance': np.zeros((3, 3), dtype=np.float32),
                'state': NOT_READY,
            }
        self.distribution_dict = EasyDict(self.distribution_dict)

    def set_accurate_distribution(self, name, data):
        '''
        Sets an accurate distribution for perceived agents
        :param name: robot name
        :param pose: robot pose
        :return:
        '''
        pose = data.pose
        mean = pose
        variance = np.zeros((3, 3), dtype=np.float32)
        variance[X, X] = 0.1
        variance[Y, Y] = 0.1
        variance[YAW, YAW] = 0.01
        state = ACCURATE
        self.distribution_dict[name].mean = mean
        self.distribution_dict[name].variance = variance
        self.distribution_dict[name].state = state

    def set_estimated_distribution(self, name, data):
        if self.distribution_dict[name].state == NOT_READY:
            # Do nothing when this robot did not appear for even once
            return
        state = ESTIMATED
        estimated_distribution = self.get_estimated_distribution(name, data)
        self.distribution_dict[name].mean = estimated_distribution.mean
        self.distribution_dict[name].variance = estimated_distribution.variance
        self.distribution_dict[name].state = state

    def get_estimated_distribution(self, name, data, step=1):
        '''
        Estimated the positioning of a robot
        :param name: robot name
        :return:
        '''
        remaining_step = step
        if step < 1:
            raise ValueError('step to request must be larger than 0!')
        else:
            # print(name, self.distribution_dict[name])
            mean_t1 = self.distribution_dict[name].mean
            variance_t1 = self.distribution_dict[name].variance
            while remaining_step > 0:
                rotation_matrix = np.array([[np.cos(mean_t1[YAW]), -np.sin(mean_t1[YAW]), 0],
                                            [np.sin(mean_t1[YAW]), np.cos(mean_t1[YAW]), 0],
                                            [0, 0, 1]], dtype=np.float32)

                # v and w in robot frame
                twist = data.twist
                # print(data)
                v = np.sqrt(twist.linear.x ** 2 + twist.linear.y ** 2)
                w = twist.angular.z
                # v = np.sqrt(mean_t1[X] ** 2 + mean_t1[Y] ** 2)
                # w = mean_t1[YAW]

                # Motion model
                variance_move = np.zeros((3, 3), dtype=np.float32)
                variance_move[X, X] = 0.1
                variance_move[Y, Y] = 0.5
                variance_move[YAW, YAW] = 0.05
                # in robot frame, move forward with v and rotate in w
                mean_move = np.array([v, 0, w])

                # Get new multivariate Gaussian
                mean_t2 = mean_t1 + np.matmul(rotation_matrix, mean_move) * self.global_config.dt
                # print('+', mean_move, '=', mean_t2)
                variance_t2 = np.matmul(rotation_matrix, variance_t1)
                variance_t2 = np.matmul(variance_t2, rotation_matrix.T)
                variance_t2 = variance_t1 + variance_t2
                mean_t1 = mean_t2.copy()
                variance_t1 = variance_t2.copy()
                remaining_step -= 1
                # if name == "robot3":
                # print(mean_t2, remaining_step)
                # print(remaining_step, '=====', name)
                # print(mean_t2, variance_t2)
                # print('==========')

            return EasyDict(
                mean=mean_t2,
                variance=variance_t2,
                step=step,
            )

    def process_observations(self, observations):
        observations = EasyDict(observations.copy())
        for obj_name, obj in observations.items():
            # print(obj_name, obj)
            if obj_name in self.distribution_dict.keys():
                # This is a robot that we want to track
                if obj.type == 'realtime':
                    # Accurate positioning
                    self.set_accurate_distribution(obj_name, obj.data)
                else:
                    self.set_estimated_distribution(obj_name)


########## Control functions ##########

def avoid(distance):
    return max(0, 0.5 - np.exp(0.05 - distance))


def get_magnitude(x):
    return np.sqrt(np.sum(x ** 2))


def braitenberg(front, front_left, front_right, left, right):
    u = 0.  # [m/s]
    w = 0.  # [rad/s] going counter-clockwise.
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
    v_avoid = get_velocity_to_avoid_obstacles(point_position, [obs.data.position for obs in observations.values() if
                                                               obs.type == 'cylinder'],
                                              [ROBOT_RADIUS + obs.data.radius for obs in observations.values() if
                                               obs.type == 'cylinder'],
                                              max_speed=max_speed)
    v = v_goal + v_avoid
    return cap(v, max_speed=max_speed)


def get_distance(x, y):
    return np.sqrt(np.sum((x - y) ** 2))


def get_unit_vector(x):
    return x / np.sqrt(np.sum(x ** 2))


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
                                    scale_factor=5,
                                    prune_distance=0.5):
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
        decay_factor = np.exp(-np.abs((to_obs_distance - obstacle_radius)) * scale_factor)

        # Assign amplitude
        if to_obs_distance <= obstacle_radius:
            amplitude = max_speed
        else:
            amplitude = decay_factor * max_speed

        if (to_obs_distance - obstacle_radius) > prune_distance:
            amplitude = 0

        # add to v. There might be multiple obstacles
        v += unit_direction * amplitude
        # print("cyliner", i, ": ", obstacle_position, position, obstacle_radius, unit_direction, amplitude)

    return v


def get_velocity_to_avoid_walls(position, wall_config, max_speed,
                                scale_factor=5):
    # This function returns velocity to avoid hitting walls
    wall_data = wall_config.params
    v = np.zeros(2, dtype=np.float32)
    SECURE_DIST = 0.5
    if wall_data.type == 'square_wall':
        def compute_velocity(distance, direction):
            if distance > 3:
                return np.zeros(2)
            # Compute the decay factor in the range of [0, 1]
            decay_factor = np.exp(- max(0, (distance - SECURE_DIST)) * scale_factor)

            # Assign amplitude
            amplitude = decay_factor * max_speed
            # print(distance, decay_factor, amplitude)
            return direction.copy() * amplitude

        left_x = wall_data.position[X]
        right_x = wall_data.position[X] + wall_data.dx
        bottom_y = wall_data.position[Y]
        top_y = wall_data.position[Y] + wall_data.dy

        # to_wall_distance should be positive within the wall

        to_wall_distance = position[X] - left_x
        sign = 1  # (position[X] - left_x) / to_wall_distance
        to_wall_direction = sign * np.array([1, 0], dtype=np.float32)
        v += compute_velocity(to_wall_distance, to_wall_direction)
        # print('left adding', compute_velocity(to_wall_distance, to_wall_direction))

        to_wall_distance = right_x - position[X]
        sign = -1  # (position[X] - right_x) / to_wall_distance
        to_wall_direction = sign * np.array([1, 0], dtype=np.float32)
        v += compute_velocity(to_wall_distance, to_wall_direction)
        # print('right adding', compute_velocity(to_wall_distance, to_wall_direction))

        to_wall_distance = position[Y] - bottom_y
        sign = 1  # (position[Y] - bottom_y) / to_wall_distance
        to_wall_direction = sign * np.array([0, 1], dtype=np.float32)
        v += compute_velocity(to_wall_distance, to_wall_direction)
        # print('bottom adding', compute_velocity(to_wall_distance, to_wall_direction))

        to_wall_distance = top_y - position[Y]
        sign = -1  # (position[Y] - top_y) / to_wall_distance
        to_wall_direction = sign * np.array([0, 1], dtype=np.float32)
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
    u = velocity[0] * np.cos(pose[YAW]) + velocity[1] * np.sin(pose[YAW])
    w = 1 / epsilon * (-velocity[0] * np.sin(pose[YAW]) + velocity[1] * np.cos(pose[YAW]))
    return u, w


def add_noise(v):
    return v + 0.1 * (np.random.random(size=2) - 0.5)


def generate_pose_msg(pose_publisher, v, point_position, frame_id):
    pose_msg = PoseStamped()
    pose_msg.header.seq = frame_id
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = 'robot1_tf/base_link'
    pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w = quaternion_from_euler(
        0, 0, np.arctan2(v[1], v[0]))
    pose_msg.pose.position.x = point_position[0]
    pose_msg.pose.position.y = point_position[1]
    pose_publisher.publish(pose_msg)
    return pose_msg


def generate_marker(marker_publisher, robot_name, v, pose, frame_id, goal=False):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.header.frame_id = 'robot1_tf/base_link'
    marker.type = marker.TEXT_VIEW_FACING
    marker.action = marker.ADD

    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    if goal:
        marker.pose.position.x = pose[0]
        marker.pose.position.y = pose[1]
        marker.text = 'G{}'.format(robot_name[-1])
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.5
    else:
        marker.text = 'R{}: {}'.format(robot_name[-1], round(np.linalg.norm(v), 2))
        # marker.text = 'R{}: {}'.format(robot_name[-1], pose.pose.position)
        marker.pose.position = pose.pose.position
        marker.color.r = 1.0
        marker.color.g = 0.9
        marker.color.b = 0.0
    marker.color.a = 1.0
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 0.3
    marker_publisher.publish(marker)
