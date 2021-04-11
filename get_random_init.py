from constant import *
import numpy as np
import rospy 
import rospkg 
from easydict import EasyDict
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
from tf.transformations import quaternion_from_euler



def initial_position(config, frame_id):
    global seed
    seed = np.random.seed(frame_id)
    m_l=[]
    p = distant_poses(config)
    for i in range(len(config.robots)):
        state_msg = ModelState()
        state_msg.model_name = 'robot'+str(i+1)
        pose = p[i]
        state_msg.pose.position.x = pose[0]
        state_msg.pose.position.y = pose[1]
        state_msg.pose.position.z = 0
        state_msg.pose.orientation.x, state_msg.pose.orientation.y, state_msg.pose.orientation.z, state_msg.pose.orientation.w = quaternion_from_euler(
        0, 0, np.random.uniform(low = 0, high = np.pi))
        state_msg.twist.linear.x = 0
        state_msg.twist.linear.y = 0
        state_msg.twist.linear.z = 0
        state_msg.twist.angular.x = 0
        state_msg.twist.angular.y = 0
        state_msg.twist.angular.z = 0
        m_l.append(state_msg)
        config.robots[i].initial_position = pose
    return m_l,config

from robotics_control import get_distance

def is_valid(pose):
    for c in CYLINDERS :
        c = EasyDict(c)
        if np.linalg.norm(pose - c.params.position) < (c.params.radius+0.5):
            return False
    return True


def random_pose(type='police'):
    low = -WALL_POSITION * 0.9
    high = WALL_POSITION * 0.9
    def get_random_pose(type='police'):
        if type == 'police':
            pose = np.concatenate([
                np.random.uniform(low=low, high=0, size=1),
                np.random.uniform(low=-2, high=2, size=1),
            ])
        else:
            pose = np.concatenate([
                np.random.uniform(low=0, high=high, size=1),
                np.random.uniform(low=low, high=high, size=1),
            ])
        return pose
    pose = get_random_pose(type=type)
    while not is_valid(pose):
        pose = get_random_pose(type=type)
    return pose

def distant_poses(config):
    p=[]
    b=[]
    r=[]
    for robot in config.robots:
        random_pos = random_pose(robot.type)
        if robot.type == 'police':
            p.append(random_pos)
        else:
            b.append(random_pos)
        r.append(random_pos)
    for pi in p:
        for bi in b:
            if np.linalg.norm(pi - bi) < WALL_POSITION*2/8 :
                return distant_poses(config)
    return r
    

    