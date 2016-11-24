#!/usr/bin/env python
"""car_pose_mux: determines car_pose (auto or manual) to use given car state.

Subscribes:
    auto_car_pose CarPose calculated from autonomous system
    manual_car_pose CarPose calculated from controller system
    car_state CarState 'auto', 'manual', 'start' or 'neutral'
Publishes:
    car_pose CarPose picked from auto/manual.
"""

import rospy

from buzzmobile.msg import CarPose, CarState


g = {} # globals
g['manual_car_pose'] = None
g['auto_car_pose'] = None
g['curr_car_state'] = CarState.START
pub = rospy.Publisher('car_pose', CarPose, queue_size=1)


def mux(car_state):
    """Returns car_pose to publish, given car_state."""
    if car_state == CarState.START: return None
    elif car_state == CarState.AUTO: return g['auto_car_pose']
    elif car_state == CarState.MANUAL: return g['manual_car_pose']

def publish():
    """Publishes current car_pose, if possible."""
    car_pose = mux(g['curr_car_state'])
    if car_pose is not None:
        pub.publish(car_pose)

def set_manual_car_pose(car_pose):
    """Updates current manual_car_pose and publishes."""
    g['manual_car_pose'] = car_pose
    publish()

def set_auto_car_pose(car_pose):
    """Updates current auto_car_pose and publishes."""
    g['auto_car_pose'] = car_pose
    publish()

def set_car_state(car_state):
    """Updates current car_state and publishes."""
    g['curr_car_state'] = car_state.state
    rospy.loginfo("Car in " + car_state_string(car_state) + " mode.")
    publish()

def car_state_string(car_state):
    """Returns string representation of current car_state."""
    if car_state.state == CarState.AUTO: return "AUTO"
    elif car_state.state == CarState.MANUAL: return "MANUAL"
    return "START"

def car_pose_mux_node():
    """Initializes car_pose_mux node."""
    rospy.init_node('car_pose_mux', anonymous=True)
    rospy.Subscriber('manual_car_pose', CarPose, set_manual_car_pose)
    rospy.Subscriber('auto_car_pose', CarPose, set_auto_car_pose)
    rospy.Subscriber('car_state', CarState, set_car_state)
    rospy.spin()

if __name__ == '__main__': car_pose_mux_node()
