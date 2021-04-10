import numpy as np
import rospy
# For groundtruth information.
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion


class GroundtruthPose(object):
    def __init__(self, global_config):
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        self.single_pose = np.array([np.nan, np.nan, np.nan], dtype=np.float32)
        self.all_pose = {}
        #self._name = config.name
        self.ready_flag = False
        self.global_config = global_config
        #self.config = config
        for obstacle in global_config.obstacles:
            self.all_pose[obstacle.name] = {'type': obstacle.params.type,
                                            'data': obstacle.params}

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
        self.ready_flag = True

    @property
    def ready(self):
        return self.ready_flag

    def pose(self, name):
        return self.all_pose[name]['data']['pose']

    def twist(self, name):
        return self.all_pose[name]['data']['twist']

    @property
    def perceived_poses(self):
        # This is all other objects poses that it can perceive
        return self.all_pose
