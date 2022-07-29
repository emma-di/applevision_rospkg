#!/usr/bin/python2

# How successful is applevision_motion? 
# Goal: run applevision_motion several times & determine success or failure

# Before starting: have rviz & simulated camera open
    # roslaunch applevision_moveit_config demo.launch
    # roslaunch applevision_rospkg fake_sensor.launch (has been condensed with demo.launch -- you don't need to run this)

# To (hopefully) run this:
    # src/applevision_rospkg/bin/dontdie.py

from random import randint
from applevision_motion import MotionPlanner, AppleApproach
from itertools import count
import rospy
import functions
import tf
import actionlib
from message_filters import Subscriber
from sensor_msgs.msg import Range
from applevision_rospkg.msg import RegionOfInterestWithConfidenceStamped, PointWithCovarianceStamped
from helpers import SynchronizerMinTick

def auto(it=count()):
    return it.next()

import logging
Logger = logging.getLogger(name="debugging the approach and motion")
handler = logging.StreamHandler()
Logger.addHandler(handler)
Logger.setLevel(logging.DEBUG)

rospy.init_node('applevision_motion')

# initial joint positions (for minimal planning problems)
joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
initial = [-3.79, -2.09, 2.15, -.052, .92, 3.87]
# apple coords
apple = [-.51, -.16, 1.3]

# if you want to get fancy, make a list of trials + results and log to a csv
# how to do with docker :(
# trials = []
Results = []
Angles = []

# setup for frame transformations (for getting coords)
listener = tf.TransformListener()

# setup for the apple approach and motion (from applevision_motion)
SYNC_SLOP = 0.2
SYNC_TICK = 0.5
camera = Subscriber('applevision/apple_camera', RegionOfInterestWithConfidenceStamped, queue_size=10)
dist = Subscriber('applevision/apple_dist', Range, queue_size=10)
kal = Subscriber('applevision/est_apple_pos', PointWithCovarianceStamped, queue_size=10)
min_tick = SynchronizerMinTick(
    [kal, camera, dist], queue_size=20, slop=SYNC_SLOP, min_tick=SYNC_TICK)

# go to home position 
# takes two parameters: list of joint names, list of joint positions
def move_to_home():
    planner.moveToJointPosition(joints, initial)

# executes apple approach (from applevision_motion)
def apple_approach():
    global min_tick
    # reset min_tick (multithreading- SynchronizerMinTick never goes away)
    min_tick.callbacks = {}
    # do the approach
    min_tick.registerCallback(approach.tick_callback)    
    # checks if process is done
    while True:
        rospy.sleep(1)
        # a small problem: sometimes it takes too long to terminate & starts running again too early
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
    
    # stop everything
    #rospy.sleep(2)
    planner.stop()
    rospy.sleep(5)
    #status = approach.planner.move_group_action.get_state()
    #print("Attempt number " + str(x+1) + ": Status " + str(status))
    #rospy.sleep(2)
    
    # check final position using transform frames
    listener.waitForTransform('/world','/palm',rospy.Time(), rospy.Duration(4.0))
    (trans, rot) = listener.lookupTransform('/world', '/palm', rospy.Time(0))

    # determine success (is palm close enough to apple?)
    if functions.nearby(trans, apple) == True:
        result = "success"
    
    # REPLACE THESE WITH THE ACTUAL THINGS AHKJASFHLKJASH
    #
    #
    #
    #
    
    # apple vector in palm camera frame
    apple_vector = [randint(5,20), 0, 0]
    # direction that palm faces (CAN BE MOVED OUT OF LOOP)
    palm_vector = [randint(5,20), 0, 0]
    
    
    
    
    
    
    
    
    
    
    
    # log results
    x+=1
    print("Number " + str(x) + " was a " + result)
    Results.append(result)
    Angles.append(functions.angle_success(apple_vector, palm_vector))
    
print(Results)
functions.get_success(Results)
functions.get_success(Angles)