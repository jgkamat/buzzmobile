#!/usr/bin/env python
import rospy

from buzzmobile.msg import CarPose
from buzzmobile.msg import CarState


g = {} # globals
g['manual_car_pose'] = None
g['auto_car_pose'] = None
g['curr_car_state'] = CarState.START

car_pose_pub = rospy.Publisher('car_pose', CarPose, queue_size=1)

def mux(car_state):
    if car_state == CarState.START: return None
    elif car_state == CarState.AUTO: return g['auto_car_pose']
    elif car_state == CarState.MANUAL: return g['manual_car_pose']

def publish():
    car_pose = mux(g['curr_car_state'])
    if car_pose is not None:
        car_pose_pub.publish(car_pose)

def set_manual_car_pose(car_pose):
    g['manual_car_pose'] = car_pose
    publish()

def set_auto_car_pose(car_pose):
    g['auto_car_pose'] = car_pose
    publish()

def set_car_state(car_state):
    g['curr_car_state'] = car_state.state
    mode = log_car_state(car_state)
    rospy.loginfo("Car in " + mode + " mode.")
    publish()

def log_car_state(car_state):
    if car_state.state == CarState.AUTO:
        return "AUTO"
    elif car_state.state == CarState.MANUAL:
        return "MANUAL"
    return "START"

def mux_node():
    rospy.init_node('car_pose_mux', anonymous=True)
    rospy.Subscriber('manual_car_pose', CarPose, set_manual_car_pose)
    rospy.Subscriber('auto_car_pose', CarPose, set_auto_car_pose)
    rospy.Subscriber('car_state', CarState, set_car_state)
    rospy.spin()

if __name__ == '__main__': mux_node()
