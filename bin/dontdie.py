#!/usr/bin/python2

# How successful is applevision_motion? 
# Goal: run applevision_motion several times & determine success or failure

# Before starting: have rviz & simulated camera open
    # roslaunch applevision_moveit_config demo.launch
    # roslaunch applevision_rospkg fake_sensor.launch

# To (hopefully) run this:
    # src/applevision_rospkg/bin/dontdie.py

from applevision_motion import MotionPlanner, AppleApproach, MoveGroupInterface
from enum import Enum#, auto
from itertools import count
def auto(it=count()):
    return it.next()
from threading import Lock
import traceback
#import sys

import rospy
import actionlib
from actionlib import GoalStatus
from message_filters import Subscriber
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, Constraints, JointConstraint, PositionConstraint, BoundingVolume, OrientationConstraint
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import Range
from tf.listener import TransformListener

from applevision_rospkg.srv import Tf2TransformPoseStamped
from applevision_rospkg.msg import RegionOfInterestWithConfidenceStamped, PointWithCovarianceStamped
from helpers import RobustServiceProxy, ServiceProxyFailed, SynchronizerMinTick

import functions
import tf

import logging
Logger = logging.getLogger(name="debugging the approach and motion")
handler = logging.StreamHandler()
Logger.addHandler(handler)
Logger.setLevel(logging.DEBUG)

rospy.init_node('applevision_motion')

# setup for frame transformations (getting coords)
#R = rospy.Rate(150)
#broadcaster = tf.TransformBroadcaster()
listener = tf.TransformListener()

# initial joint positions
joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
initial = [0, -.58, -2.26, -.44, 1.62, .74]
# apple coords
apple = [-.51, -.16, 1.3]

bot = MotionPlanner()
planner = MotionPlanner()
approach = AppleApproach(planner)

GROUP_NAME = 'manipulator'
EE_FRAME = 'palm'
WORLD_FRAME = 'world'
PLANNING_TIME = 5.0
SYNC_SLOP = 0.2
SYNC_TICK = 0.5
MOVE_TOLERANCE = 0.03

camera = Subscriber('applevision/apple_camera', RegionOfInterestWithConfidenceStamped, queue_size=10)
dist = Subscriber('applevision/apple_dist', Range, queue_size=10)
kal = Subscriber('applevision/est_apple_pos', PointWithCovarianceStamped, queue_size=10)

# if you want to get fancy, make a list of trials + results and log to a csv
# how to do with docker :(
# trials = []
# results = []

# loops through given number of times
for x in range(int(input("Run how many times? "))):
    # reset
    result = "fail"

    # go to home position
    bot.moveToJointPosition(joints, initial)
    #print("move joints done")
    
    # the actual apple approach
    min_tick = SynchronizerMinTick(
        [kal, camera, dist], queue_size=20, slop=SYNC_SLOP, min_tick=SYNC_TICK)
    min_tick.registerCallback(approach.tick_callback)
    
    print("START SLEEPING")
    rospy.sleep(120)
    print("LOOK HERE")
    # bot = applevision_motion.MoveGroupInterface()
    # bot.moveToJointPosition(joints, initial)
    
    # check final position
    listener.waitForTransform('/world','/palm',rospy.Time(), rospy.Duration(4.0))
    (trans, rot) = listener.lookupTransform('/world', '/palm', rospy.Time(0))

    # determine success (is palm close enough to apple?)
    if functions.nearby(trans, apple) == True:
        result = "success"

    x+=1
    
    # log results (see note above)
    print("Number " + str(x) + " was a " + result)