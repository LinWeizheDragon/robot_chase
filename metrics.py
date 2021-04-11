import numpy as np
from easydict import EasyDict
import rospy

class MetricsManager():
    def __init__(self, config):
        self.config=config
        self.instance_dict=None
        self.all_success = False
        self.individual_success_list = []
        self.capture_time = {}
        self.capture_history = []
        self.logs = []
        self.frame_id = 0
        self.last_update_timestamp = rospy.get_rostime()

    def get_log_data(self):
        '''
        Return log data
        :return:
            logs: [print1, print2, ...]
            total_frames: int
            capture_history: [EasyDict(
                            police={
                                police_1: (baddy_1, ...),
                                police_2: (baddy_3, ...) ,
                            },
                            baddies={
                                baddy_2: (),
                                baddy_1: (police_1, ...),
                                baddy_3: (police_2, ...),
                            },
                        )]
            all_success: bool
        '''
        return EasyDict(
            logs=self.logs,
            total_frames=self.frame_id,
            total_time=rospy.get_rostime().to_sec(),
            capture_history=self.capture_history,
            all_success=self.all_success,
            individual_success_list=self.individual_success_list,
            capture_time=self.capture_time,
        )

    def set_instance_dict(self, instance_dict):
        self.instance_dict = instance_dict

    def add_log(self, *args):
        '''
        Add print logs to history
        :param args: a sequence of args, as in print()
        :return:
        '''
        self.logs.append((rospy.get_rostime().to_sec(), self.frame_id, args))

    def update(self, frame_id):
        '''
        Update current state to the history
        :param frame_id: int current frame
        :return:
        '''
        self.frame_id = frame_id
        self.last_update_timestamp = rospy.get_rostime()
        current_state = EasyDict(
            timestamp=rospy.get_rostime().to_sec(),
            police={},
            baddies={},
        )
        all_success = True
        success_list = []
        for robot_name, robot_instance in self.instance_dict.items():
            if robot_instance.type=='police':
                # print(robot_instance.name, robot_instance.captured)
                current_state.police[robot_name] = robot_instance.captured
            else:
                # print(robot_instance.name, robot_instance.capture_by)
                if len(robot_instance.capture_by) == 0:
                    # this baddy has not been captured by anyone
                    # print('baddy', robot_instance.name, 'is not captured yet.')
                    all_success = False
                    success_list.append(False)
                else:
                    success_list.append(True)
                    if self.capture_time.get(robot_name, None) is None:
                        self.capture_time[robot_name] = rospy.get_rostime().to_sec()

                current_state.baddies[robot_name] = robot_instance.capture_by

        self.capture_history.append(current_state)
        self.all_success = all_success
        self.individual_success_list = success_list

