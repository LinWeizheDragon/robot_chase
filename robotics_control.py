
# Robot motion commands:
# http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
from geometry_msgs.msg import Twist
import numpy as np

class RobotAbstract():
    def __init__(self, publisher, global_config, config, sensor, positioning):
        self.sensor = sensor
        self.publisher = publisher
        self.type = config.type
        self.name = config.name
        self.positioning = positioning

    def action(self):
        measurements = self.sensor.measurements
        u, w = self.controller(*measurements)
        vel_msg = Twist()
        vel_msg.linear.x = u
        vel_msg.angular.z = w
        self.publisher.publish(vel_msg)
        print('robot', self.name, 'actioned')

    def controller(self, *args, **kwargs):
        raise NotImplementedError('No Controller is specified!')

    def get_current_position(self):
        return self.positioning.pose

class Police(RobotAbstract):
    def __init__(self, publisher, global_config, config, sensor, positioning):
        RobotAbstract.__init__(self, publisher, global_config, config, sensor, positioning)

    def controller(self, *args, **kwargs):
        u, w = braitenberg(*args)
        return u, w


class Baddy(RobotAbstract):
    def __init__(self, publisher, global_config, config, sensor, positioning):
        RobotAbstract.__init__(self, publisher, global_config, config, sensor, positioning)

    def controller(self, *args, **kwargs):
        u, w = braitenberg(*args)
        return u, w


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

