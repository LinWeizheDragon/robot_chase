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
SECURITY_DISTANCE = EasyDict({"competitor": 3, "obstacle": 0, "companian": 0})
print(SECURITY_DISTANCE)
CAPTURE_DISTANCE = 1