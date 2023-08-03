#!/usr/bin/env python2
# For miscellaneous functions
# To run this:
    # src/applevision_rospkg/bin/functions.py

from math import acos, degrees, sqrt
from datetime import datetime
from pytz import timezone
import pytz

import os
import shutil
# functions to remove and move folders and files
#shutil.rmtree('/root/data/08-26-2022 14:33')
#shutil.move('/root/data/stage1_{}_2022-{}.csv'.format(100,'08-22 23:16'), '/root/data/old/{}_2022-{}.csv'.format(100,'08-22 23:16'))
#os.remove('/root/data/stage1_2022-08-18 {}.csv'.format('19:47:59.215964'))

# calculating distance between apple and endeffector
# takes two arrays ([x,y,z]) for positions of apple and endeffector 
def nearby(real, apple):
    # calculates distance and height difference
    dstance = sqrt((abs(real[0]-apple[0]))**2+(abs(real[1]-apple[1]))**2+(abs(real[2]-apple[2]))**2)
    heightdiff = abs(real[2]-apple[2])
    xdiff = abs(real[0]-apple[0])
    # check is distance is height difference is acceptable
    if dstance < .15 and heightdiff <.05 and xdiff < .05:
        return True

# convert datetime to PST
def current_time():
    date_format='%Y-%m-%d %H:%M'
    date = datetime.now(tz=pytz.utc)
    date = date.astimezone(timezone('US/Pacific'))
    return date.strftime(date_format)

# return % success
def get_success(results):    
    successes = 0
    for result in results:
        if result in ["success", True]:
            successes += 1
    x = len(results)-1
    return (str(round((successes/float(x))*100, 2)) + " percent")

# computes the magnitude of a vector    
def magnitude(vector):
    return float(sqrt(vector[0]**2+vector[1]**2+vector[2]**2))
    
# takes two vectors and determines the angle between them    
def angle_success(v1, v2):
    dotproduct = float(v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2])
    angle = degrees(acos(dotproduct/(magnitude(v1)*magnitude(v2))))
    if angle <= 15:
        return True, round(angle,2)
    else:
        return False, round(angle, 2)

# gets average of values in a list from a certain point to the end
# can choose to disregard any values outside of outlier threshold
#TODO: hacky, maybe find a way to automatically determine bounds (use IQR?)
def average_value(list, start, lower_bound, upper_bound):
    sum = 0
    count = 0
    for x in range (start, len(list)):
        # disregards outliers
        if float(list[x]) > lower_bound:
            if float(list[x]) < upper_bound:
                sum = sum + float(list[x])
                count += 1
    return(round(float(sum/count), 2))

def get_occlusion(img):
    pass

# variables across all stages for easy updating
runs = input("Run how many times? ")
joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
# initial = [-3.63, -2.09, 2.15, -.28, .92, 4.57]
# initial = [-3.79, -2.09, 2.15, -.28, .92, 4.35]
initial = [-3.278, -2.16, 1.93, .2046, 1.36, 3.94] #(in degrees: roughly -180, -121, 108, 13, 90, 226)