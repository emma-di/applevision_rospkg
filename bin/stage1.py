#!/usr/bin/python2

# How successful is applevision_motion? 
# Goal: run applevision_motion several times & determine success or failure

# Before starting: have rviz & simulated camera open
    # roslaunch applevision_moveit_config demo.launch
    # roslaunch applevision_rospkg fake_sensor.launch (has been condensed with demo.launch -- you don't need to run this)

# To (hopefully) run this:
    # src/applevision_rospkg/bin/stage1.py
    
from applevision_motion import MotionPlanner, AppleApproach
from itertools import count
import rospy
import functions
import tf
import time
import csv
import os
import numpy as np
from message_filters import Subscriber
from sensor_msgs.msg import Range
from datetime import datetime
from applevision_rospkg.msg import RegionOfInterestWithConfidenceStamped, PointWithCovarianceStamped
from helpers import SynchronizerMinTick
from scipy.spatial.transform import Rotation as R
from data_visualizations import visualizations

def auto(it=count()):
    return it.next()

import logging
Logger = logging.getLogger(name="debugging the approach and motion")
handler = logging.StreamHandler()
Logger.addHandler(handler)
Logger.setLevel(logging.DEBUG)

rospy.init_node('applevision_motion')
runs = input("Run how many times? ")

# initial joint positions (for minimal planning problems)
joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
initial = [-3.79, -2.09, 2.15, -.052, .92, 3.87]
# apple coords
apple = [-.51, -.16, 1.3]
# direction the palm faces
palm_vector = [0,0,.1]

# storing results
Trials = ['TRIAL:']
Results = ['RESULT:']
Angles = ['ANGLE:']
Angle_Log = ['']
Approach_Times = ['APPROACH TIME:']
Apple_Vectors = ['VECTOR:']

# csv with results
current_time = functions.current_time()
os.mkdir('/root/data/{}'.format(current_time))
with open('/root/data/{}/stage1_{}.csv'.format(current_time, runs), 'w') as spreadsheet:
    writer = csv.writer(spreadsheet)

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
    # reset min_tick (multithreading -- SynchronizerMinTick never goes away)
    min_tick.callbacks = {}
    # do the approach
    min_tick.registerCallback(approach.tick_callback)    
    # checks if process is done
    while True:
        rospy.sleep(1)
        # a small problem: sometimes it takes too long to terminate & starts running again too early
        if approach.is_done() == True:
            return
            
# loops through given number of times
for x in range(int(runs)):
    # reset
    result = "fail"

    # create objects for apple approach
    planner = MotionPlanner()
    approach = AppleApproach(planner)
    
    # go to home position
    move_to_home()

    # approach the apple (and log how long it takes)
    start = time.time()
    kal = apple_approach()
    end = time.time()
    approach_time = round(end-start,2)
    Approach_Times.append(approach_time)
    
    # stop everything
    planner.stop()
    # ensure that is is stopped (sometimes it gets stuck)
    planner.stop()
    rospy.sleep(10)
    planner.stop()
    rospy.sleep(10)
    
    # check final position using transform frames
    listener.waitForTransform('/world','/palm',rospy.Time(), rospy.Duration(4.0))
    (trans, rot) = listener.lookupTransform('/world', '/palm', rospy.Time(0))
    r = R.from_quat(rot)

    # determine success (is palm close enough to apple?)
    if functions.nearby(trans, apple) == True:
        result = "success"
    
    # apple vector in palm camera frame
    apple_array = np.array(apple)
    trans_array = np.array(trans)
    apple_vector = r.apply(trans_array-apple_array)
    Apple_Vectors.append(apple_vector)
    print("this is the apple: " + str(apple_vector))
 
    # log results
    x+=1
    print("Number " + str(x) + " was a " + result)
    Trials.append(x)
    Results.append(result)
    Angles.append(functions.angle_success(apple_vector, palm_vector)[1])
    Angle_Log.append(functions.angle_success(apple_vector, palm_vector)[0])
    with open('/root/data/{}/stage1_{}.csv'.format(current_time, runs), 'w') as spreadsheet:
        writer = csv.writer(spreadsheet)
        writer.writerow(Trials)
        writer.writerow(Results)
        writer.writerow(Angles)
        writer.writerow(Apple_Vectors)
        writer.writerow(Approach_Times)

# calculate success
Trials.insert(0, '')
Results_success = functions.get_success(Results)
Results.insert(0, Results_success)
Angles_success = functions.get_success(Angle_Log)
Angles.insert(0, Angles_success)
Apple_Vectors.insert(0, '')
Average_Time = functions.average_value(Approach_Times, 1, 15, 40) # excludes failed approach times
Approach_Times.insert(0, "Average time: "+str(Average_Time))

# log results to csv
with open('/root/data/{}/stage1_{}.csv'.format(current_time, runs), 'w') as spreadsheet:
        writer = csv.writer(spreadsheet)
        writer.writerow(Trials)
        writer.writerow(Results)
        writer.writerow(Angles)
        writer.writerow(Apple_Vectors)
        writer.writerow(Approach_Times)
# data visualizations
visualizations('/root/data/{}/stage1_{}.csv'.format(current_time, runs))