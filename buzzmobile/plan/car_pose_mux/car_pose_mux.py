#!/usr/bin/env python
import rospy
from buzzmobile.msg import CarPose
from buzzmobile.msg import CarState

#Global Variables
controller_car_pose = None
auto_car_pose = None
curr_car_state = None

car_pose_pub = rospy.Publisher('car_pose', CarPose, queue_size=0)

def publish():
    if curr_car_state != None: 
        car_pose = CarPose()
        if (curr_car_state.state == CarState.AUTO):
            car_pose = auto_car_pose
        else:
            car_pose = controller_car_pose
        
        if (car_pose != None):
            rospy.loginfo("Car in " + ("AUTO" if curr_car_state.state == CarState.AUTO else "MANUAL") + " mode.")
            car_pose_pub.publish(car_pose)

def set_controller_car_pose(car_pose):
    global controller_car_pose
    controller_car_pose = car_pose

def set_auto_car_pose(car_pose):
    global auto_car_pose
    auto_car_pose = car_pose

def set_car_state(car_state):
    global curr_car_state
    curr_car_state = car_state
    publish()

def mux_node():
    rospy.init_node('car_pos_mux', anonymous=True)
    rospy.Subscriber('controller_car_pose', CarPose, set_controller_car_pose)
    #rospy.Subscriber('auto_car_pose', CarPose, set_auto_car_pose)
    rospy.Subscriber('car_state', CarState, set_car_state)
    rospy.spin()

if __name__ == '__main__':
    mux_node()
