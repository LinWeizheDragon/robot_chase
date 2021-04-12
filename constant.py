'''
This file defines constant numbers
'''

import numpy as np
from easydict import EasyDict

X = 0
Y = 1
YAW = 2

ROBOT_RADIUS = 0.105 / 2.
GOAL_POSITION = np.array([-0.5, -0.5], dtype=np.float32)
SECURITY_DISTANCE = EasyDict({"competitor": 7, "obstacle": 0, "companion": 1})
print(SECURITY_DISTANCE)
CAPTURE_DISTANCE = 0.7
WALL_POSITION = 8.64
CYLINDERS =  ([{
                    'name': 'cylinder1',
                    'params': {
                        'type': 'cylinder',
                        'position': np.array([3, 2], dtype=np.float32),
                        'radius': 1.5,
                    }
                },
                {
                    'name': 'cylinder2',
                    'params': {
                        'type': 'cylinder',
                        'position': np.array([-1, -5.5], dtype=np.float32),
                        'radius': 1.3,
                    }
                },
                {
                    'name': 'cylinder3',
                    'params': {
                        'type': 'cylinder',
                        'position': np.array([-4, 3.5], dtype=np.float32),
                        'radius': 1.2,
                    }
                }])