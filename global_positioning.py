
import numpy as np
import rospy
# For groundtruth information.
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion

class GroundtruthPose(object):
  def __init__(self, config):
    rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
    self.single_pose = np.array([np.nan, np.nan, np.nan], dtype=np.float32)
    self._name = config.name
    self.ready_flag = False

  def callback(self, msg):
    idx = [i for i, n in enumerate(msg.name) if n == self._name]
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
    self.ready_flag = True

  @property
  def ready(self):
    return self.ready_flag

  @property
  def pose(self):
    return self.single_pose
