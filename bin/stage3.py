#!/usr/bin/python2

# Stage 3 goal: verify that it can identify and approach apples despite occlusion
    
# Running it with the real robot:
    # 1: roslaunch applevision_moveit_config realrobot.launch
        # wait until it's ready, then press play on the UR control panel
    # 5: src/applevision_rospkg/bin/stage3.py
    
from applevision_motion import MotionPlanner, AppleApproach
from itertools import count
import rospy
import functions
import tf
import time
import csv
import os
import numpy as np
import random
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
runs = functions.runs
pct = input("Percent occlusion? ")

# initial joint positions (for minimal planning problems)
joints = functions.joints
initial = functions.initial

# setup for frame transformations (for getting coords)
listener = tf.TransformListener()
# apple coords
listener.waitForTransform('/world','/apple',rospy.Time(), rospy.Duration(4.0))
(apple, rot) = listener.lookupTransform('/world', '/apple', rospy.Time(0))
# direction the palm faces
palm_vector = [0,0,.1]
trial = 1

# storing results
Trials = ['TRIAL:']
Results = ['RESULT:']
Angles = ['ANGLE:']
Angle_Log = ['']
Approach_Times = ['APPROACH TIME:']
Apple_Vectors = ['VECTOR:']
Start_Coords = ['START COORDS:']
Max_Conf = ['MAX CONF:']
Avg_Conf = ['AVG CONF:']

# csv with results
current_time = functions.current_time()
os.mkdir('/root/data/{}'.format(current_time))
with open('/root/data/{}/stage3_{}_{}.csv'.format(current_time, runs, pct), 'w') as spreadsheet:
    writer = csv.writer(spreadsheet)

# setup for the apple approach and motion (from applevision_motion)
SYNC_SLOP = 0.2
SYNC_TICK = 0.5
camera = Subscriber('applevision/apple_camera', RegionOfInterestWithConfidenceStamped, queue_size=10)
dist = Subscriber('gripper/dist', Range, queue_size=10)
kal = Subscriber('applevision/est_apple_pos', PointWithCovarianceStamped, queue_size=10)
min_tick = SynchronizerMinTick(
    [kal, camera, dist], queue_size=20, slop=SYNC_SLOP, min_tick=SYNC_TICK)

# go to home position 
# takes two parameters: list of joint names, list of joint positions

# def move_to_home():
#     planner.moveToJointPosition(joints, initial)

# executes apple approach (from applevision_motion)
def apple_approach(approach):
    global min_tick
    # reset min_tick (multithreading -- SynchronizerMinTick never goes away)
    min_tick.callbacks = {}
    # do the approach (approach gets defined in the loop)
    min_tick.registerCallback(approach.tick_callback)    
    # checks if process is done
    while True:
        rospy.sleep(1)
        # a small problem: sometimes it takes too long to terminate & starts running again too early
        if approach.is_done() == True:
            return
            
    # loops through given number of times
def loop_approach():
    rospy.sleep(5)
    
    # reset
    result = "fail"
    success = False

    # create objects for apple approach
    planner = MotionPlanner()
    
    # go to home position
    planner.moveToJointPosition(joints, initial)
    planner.start_move_to_pose((0,0,0), .03)
    # stage 2 testing is seeing if it can approach from various starting positions
    startcoords = [(random.random()-0.5)/5, (random.random()-0.5)/5]
    Start_Coords.append(startcoords)
    print("start pos: " + str(startcoords))
    planner.start_move_to_pose((startcoords[0],startcoords[1], 0), .03)
    rospy.sleep(2.5)

    # approach the apple (and log how long it takes)
    approach1 = AppleApproach(planner)
    start = time.time()
    apple_approach(approach1)
    end = time.time()
    approach_time = round(end-start,2)
    Approach_Times.append(approach_time)
    
    # record conf data
    approach1.conf_summary
    # (max_conf, avg_conf) = approach1.conf_summary
    # Max_Conf.append(max_conf)
    # Avg_Conf.append(avg_conf)
    
    # stop everything
    # ensure that is is stopped (sometimes it gets stuck)
    planner.stop()
    rospy.sleep(10)
    planner.stop()
    
    # check final position using transform frames
    listener.waitForTransform('/world','/palm',rospy.Time(), rospy.Duration(4.0))
    (trans, rot) = listener.lookupTransform('/world', '/palm', rospy.Time(0))
    r = R.from_quat(rot)

    # determine success (is palm close enough to apple?)
    if functions.nearby(trans, apple) == True:
        result = "success"
        success = True
    print("this trial was a " + result)
    
    # apple vector in palm camera frame
    apple_array = np.array(apple)
    trans_array = np.array(trans)
    apple_vector = r.apply(trans_array-apple_array)
    Apple_Vectors.append(apple_vector)
    print("this is the apple: " + str(apple_vector))

    # log results (loggin now in case it gets stuck or doesn't finish)
    Results.append(result)
    Angles.append(functions.angle_success(apple_vector, palm_vector)[1])
    Angle_Log.append(functions.angle_success(apple_vector, palm_vector)[0])
    with open('/root/data/{}/stage3_{}_{}.csv'.format(current_time, runs, pct), 'w') as spreadsheet:
        writer = csv.writer(spreadsheet)
        writer.writerow(Trials)
        writer.writerow(Results)
        writer.writerow(Angles)
        writer.writerow(Apple_Vectors)
        writer.writerow(Approach_Times)
        writer.writerow(Start_Coords)
        writer.writerow(Max_Conf)
        writer.writerow(Avg_Conf)
    return success
            
for x in range(int(runs)):
    loop_approach()
    Trials.append(x)

#TODO: make a function for this
# calculate success
Trials.insert(0, '')
Results_success = functions.get_success(Results)
Results.insert(0, Results_success)
Angles_success = functions.get_success(Angle_Log)
Angles.insert(0, Angles_success)
Apple_Vectors.insert(0, '')
Average_Time = functions.average_value(Approach_Times, 1, 6, 55) # excludes failed approach times
Approach_Times.insert(0, str(Average_Time))
Start_Coords.insert(0, '')
Max_Conf.insert(0,'')
Avg_Conf.insert(0,'')

# log results to csv
with open('/root/data/{}/stage3_{}_{}.csv'.format(current_time, runs, pct), 'w') as spreadsheet:
        writer = csv.writer(spreadsheet)
        writer.writerow(Trials)
        writer.writerow(Results)
        writer.writerow(Angles)
        writer.writerow(Apple_Vectors)
        writer.writerow(Approach_Times)
        writer.writerow(Start_Coords)
        writer.writerow(Max_Conf)
        writer.writerow(Avg_Conf)
# data visualizations
visualizations('/root/data/{}/stage3_{}_{}.csv'.format(current_time, runs, pct))