import numpy as np
import rospy
# For groundtruth information.
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
from robotics_control import get_distance

FULL_VISION_TIME = 3

class GroundtruthPose(object):
    def __init__(self, global_config):
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        self.single_pose = np.array([np.nan, np.nan, np.nan], dtype=np.float32)
        self.all_pose = {} # groundtruth poses
        self.individual_pose = {} # poses that can be perceived by each robot
        self.baddy_communicated = []
        self.ready_flag = False
        self.global_config = global_config
        for obstacle in global_config.obstacles:
            self.all_pose[obstacle.name] = {'type': obstacle.params.type,
                                            'data': obstacle.params}
        for robot in self.global_config.robots:
            # update obstacle info to individual perception
            self.individual_pose[robot.name] = self.all_pose.copy()
        self.award_time_update = rospy.get_rostime()

    def reset_award_time(self):
        self.award_time_update = rospy.get_rostime()

    def callback(self, msg):
        # idx = [i for i, n in enumerate(msg.name)]
        # if not idx:
        #   raise ValueError('Specified name "{}" does not exist.'.format(self._name))
        for i, name in enumerate(msg.name):

            obj_pose = np.array([np.nan, np.nan, np.nan], dtype=np.float32)
            obj_pose[0] = msg.pose[i].position.x
            obj_pose[1] = msg.pose[i].position.y
            _, _, yaw = euler_from_quaternion([
                msg.pose[i].orientation.x,
                msg.pose[i].orientation.y,
                msg.pose[i].orientation.z,
                msg.pose[i].orientation.w])
            obj_pose[2] = yaw
            #if name == self._name:
            #    self.single_pose = obj_pose.copy()
            self.all_pose[name] = {
                'type': 'realtime',
                'data': {
                    'pose': obj_pose.copy(),
                    'twist': msg.twist[i],
                },
            }
            self.baddy_communicated = []
        self.ready_flag = True

    @property
    def ready(self):
        return self.ready_flag

    def pose(self, name):
        return self.all_pose[name]['data']['pose']

    def twist(self, name):
        return self.all_pose[name]['data']['twist']

    def get_perceived_poses(self, query_name=None, visibility=0):
        # This is all other objects poses that it can perceive
        if query_name is None:
            return self.all_pose
        else:
            if visibility == 0:
                # 0 means full vision
                return self.all_pose

            if (rospy.get_rostime()-self.award_time_update).to_sec() < FULL_VISION_TIME:
                # within FULL_VISION_TIME seconds, allow the police to see baddies
                self.individual_pose[query_name].update(self.all_pose.copy())
                return self.all_pose

            query_type = [robot.type for robot in self.global_config.robots if robot.name == query_name][0]

            if query_type == 'baddy':
                # baddy can always see everything
                self.individual_pose[query_name].update(self.all_pose.copy())
                return self.all_pose


            query_pose = self.all_pose[query_name]['data']['pose']

            robot_needs_to_be_seen = []

            for robot in self.global_config.robots:
                if robot.name == query_name:
                    # update accurate positioning for the query robot
                    self.individual_pose[robot.name][robot.name] = self.all_pose[robot.name].copy()
                else:
                    # other robots, check distance
                    # from pprint import pprint
                    # pprint(self.individual_pose)
                    other_robot_pose = self.all_pose[robot.name]['data']['pose']
                    if get_distance(other_robot_pose[:2], query_pose[:2]) > visibility and robot.name not in self.baddy_communicated:
                        # query robot can not see this robot
                        # use last time observations
                        # print(query_name, 'can not see', robot.name)
                        self.individual_pose[query_name][robot.name]['type'] = 'history'
                        self.individual_pose[query_name][robot.name]['data'].setdefault('past', 0)
                        self.individual_pose[query_name][robot.name]['data']['past'] += 1
                    else:
                        # query robot can see this robot
                        # update observations
                        self.individual_pose[query_name][robot.name] = self.all_pose[robot.name].copy()
                        robot_needs_to_be_seen.append(robot.name)

                        # Communication, comment out to remove
                        self.baddy_communicated.append(robot.name)
            return self.individual_pose[query_name]