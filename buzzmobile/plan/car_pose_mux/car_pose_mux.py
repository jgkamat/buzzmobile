#!/usr/bin/env python
import rospy

from buzzmobile.msg import CarPose
from buzzmobile.msg import CarState


# Global Variables
state = {
        'manual_car_pose': None,
        'auto_car_pose': None,
        'curr_car_state': CarState.START}

car_pose_pub = rospy.Publisher('car_pose', CarPose, queue_size=0)

def mux(car_state):
    if car_state == CarState.START: return None
    elif car_state == CarState.AUTO: return state['auto_car_pose']
    elif car_state == CarState.MANUAL: return state['manual_car_pose']

def publish():
    car_pose = mux(state['curr_car_state'])
    if car_pose is not None:
        car_pose_pub.publish(car_pose)

def set_manual_car_pose(car_pose):
    state['manual_car_pose'] = car_pose
    publish()

def set_auto_car_pose(car_pose):
    state['auto_car_pose'] = car_pose
    publish()

def set_car_state(car_state):
    state['curr_car_state'] = car_state.state
    mode = ""
    if car_state.state == CarState.AUTO:
        mode = "AUTO"
    elif car_state.state == CarState.MANUAL:
        mode = "MANUAL"
    else:
        mode = "START"
    rospy.loginfo("Car in " + mode + " mode.")
    publish()

def mux_node():
    rospy.init_node('car_pose_mux', anonymous=True)
    rospy.Subscriber('manual_car_pose', CarPose, set_manual_car_pose)
    rospy.Subscriber('auto_car_pose', CarPose, set_auto_car_pose)
    rospy.Subscriber('car_state', CarState, set_car_state)
    rospy.spin()

if __name__ == '__main__': mux_node()
