#!/usr/bin/python2

# How successful is applevision_motion? 
# Goal: run applevision_motion several times & determine success or failure

# Before starting: have rviz & simulated camera open
    # roslaunch applevision_moveit_config demo.launch
    # roslaunch applevision_rospkg fake_sensor.launch (has been condensed with demo.launch -- you don't need to run this)

# To (hopefully) run this:
    # src/applevision_rospkg/bin/dontdie.py

from applevision_motion import MotionPlanner, AppleApproach
from itertools import count
import rospy
import functions
import tf
import actionlib
from message_filters import Subscriber
from sensor_msgs.msg import Range
from applevision_rospkg.msg import RegionOfInterestWithConfidenceStamped, PointWithCovarianceStamped
from helpers import RobustServiceProxy, ServiceProxyFailed, SynchronizerMinTick

def auto(it=count()):
    return it.next()

import logging
Logger = logging.getLogger(name="debugging the approach and motion")
handler = logging.StreamHandler()
Logger.addHandler(handler)
Logger.setLevel(logging.DEBUG)

rospy.init_node('applevision_motion')

# initial joint positions
joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
initial = [-3.79, -2.09, 2.15, -.052, .92, 3.87]
# apple coords
apple = [-.51, -.16, 1.3]


# setup for frame transformations (getting coords)
listener = tf.TransformListener()

# setup for the apple approach and motion (from applevision_motion)
SYNC_SLOP = 0.2
SYNC_TICK = 0.5
camera = Subscriber('applevision/apple_camera', RegionOfInterestWithConfidenceStamped, queue_size=10)
dist = Subscriber('applevision/apple_dist', Range, queue_size=10)
kal = Subscriber('applevision/est_apple_pos', PointWithCovarianceStamped, queue_size=10)
min_tick = SynchronizerMinTick(
    [kal, camera, dist], queue_size=20, slop=SYNC_SLOP, min_tick=SYNC_TICK)

# if you want to get fancy, make a list of trials + results and log to a csv
# how to do with docker :(
# trials = []
results = []

# go to home position 
# takes two parameters: list of joint names, list of joint positions
def move_to_home():
    planner.moveToJointPosition(joints, initial)

# executes apple approach (from applevision_motion)
def apple_approach():
    global min_tick
    min_tick.callbacks = {}
    min_tick.registerCallback(approach.tick_callback)
    # checks if process is done
    while True:
        rospy.sleep(1)
        if approach.is_done() == True:
            break

# loops through given number of times
for x in range(int(input("Run how many times? "))):
    # reset
    result = "fail"

    # create objects for apple approach
    planner = MotionPlanner()
    approach = AppleApproach(planner)
    
    # go to home position
    move_to_home()
    
    # approach the apple
    apple_approach()
    
    #rospy.sleep(2)
    planner.stop()
    #rospy.sleep(2)
    status = approach.planner.move_group_action.get_state()
    #print("Attempt number " + str(x+1) + ": Status " + str(status))
    #rospy.sleep(2)
    
    # check final position using transform frames
    listener.waitForTransform('/world','/palm',rospy.Time(), rospy.Duration(4.0))
    (trans, rot) = listener.lookupTransform('/world', '/palm', rospy.Time(0))

    # determine success (is palm close enough to apple?)
    if functions.nearby(trans, apple) == True:
        result = "success"
    
    # log results
    x+=1
    print("Number " + str(x) + " was a " + result)
    results.append(result)

print(results)