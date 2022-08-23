#!/usr/bin/env python2
#for testing random things
# To run this:
    # src/applevision_rospkg/bin/functions.py

from math import acos, degrees, sqrt
# from geometry_msgs.msg import PointStamped
import rospy
# import applevision_motion
# from tf.listener import TransformListener
# import moveit_commander
# import sys
# from geometry_msgs.msg import Pose, PoseStamped
# from moveit_msgs.msg import RobotTrajectory, Grasp, PlaceLocation, Constraints
# from sensor_msgs.msg import JointState
import rospy
import tf
import csv
# from moveit_ros_planning_interface import _moveit_move_group_interface
# from exception import MoveItCommanderException
# from datetime import datetime
import matplotlib.pyplot as plt
import numpy as np

# not used anymore... did waitForTransform directly in dontdie.py instead
def getpose():
    #rospy.init_node('applevision_motion')
    # now = rospy.Time.now()
    R = rospy.Rate(150)
    # while not rospy.is_shutdown():
    broadcaster = tf.TransformBroadcaster()
    #broadcaster.sendTransform((1, 1, 1), (0, 0, 0, 1), rospy.Time(), '/world','/palm')      
    R.sleep()

    listener = tf.TransformListener()
    #now = rospy.Time.now()
    listener.waitForTransform('/world','/palm',rospy.Time(), rospy.Duration(4.0))
    (trans, rot) = listener.lookupTransform('/world', '/palm', rospy.Time(0))
    return (trans)

#.0832949 (apple height)
#things measured in meters

# calculating distance between apple and endeffector (this needs getpose())
# takes two arrays ([x,y,z]) for positions of apple and endeffector 
def nearby(real, apple):
    # calculates distance and height difference
    dstance = sqrt((abs(real[0]-apple[0]))**2+(abs(real[1]-apple[1]))**2+(abs(real[2]-apple[2]))**2)
    heightdiff = abs(real[2]-apple[2])
    # check is distance is height difference is acceptable
    if dstance < .15 and heightdiff <.05:
        return True

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
    if angle <= 5:
        return True, round(angle,2)
    else:
        return False, round(angle, 2)

# gets average of values in a list from a certain point to the end
# can choose to disregard any values under outlier threshold
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

# for angle/vector visualization (copied from https://matplotlib.org/stable/gallery/lines_bars_and_markers/scatter_hist.html#sphx-glr-gallery-lines-bars-and-markers-scatter-hist-py)
def scatter_hist(axis1, axis2, ax, ax_histx, ax_histy):
    # no labels
    ax_histx.tick_params(axis=str(axis1), labelbottom=False)
    ax_histy.tick_params(axis=str(axis2), labelleft=False)

    # the scatter plot:
    ax.scatter(axis1, axis2)

    # now determine nice limits by hand:
    binwidth = 0.25
    xymax = max(np.max(np.abs(axis1)), np.max(np.abs(axis2)))
    lim = (int(xymax/binwidth) + 1) * binwidth

    bins = np.arange(-lim, lim + binwidth, binwidth)
    ax_histx.hist(axis1, bins=bins)
    ax_histy.hist(axis2, bins=bins, orientation='horizontal')

#lllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllll

# class for data visualization
# takes one parameter: data spreadsheet location as a string (spreadsheet from stagex testing files)
class DataVis():
    
    # label and sort data
    def __init__(self, datasheet):
        self.name = datasheet[11:27]
        with open (datasheet) as f:
            reader = csv.reader(f)
            self.data = list(reader)
            self.end = len(self.data[0])
            self.trials = (self.data[0])[2:self.end]
            self.results = (self.data[1])[2:self.end]
            self.angles = (self.data[2])[2:self.end]
            self.vectors = (self.data[3])[2:self.end]
            times = (self.data[4])[2:self.end]
            self.times = [eval(i) for i in times]
            #get average time without fails THIS IS BAD I DONT WANT THIS HERE I WANT IT UP THERE
            sum = 0
            count = 0
            for i in range (len(self.times)):
                if self.results[i] == 'success':
                    if not(self.times[i] > 40 or self.times[i]<15):
                        sum = sum + float(self.times[i])
                        count += 1
            self.avg_time = (round(float(sum/count), 2))  
    # visualization for approach times (scatter plot + avg time line)
    def time_vis(self):
        # search for outliers
        normal_times = []
        normal_trials = []
        strange_times = []
        strange_trials = []
        outlier_times = []
        outlier_trials = []
        for i in range(len(self.times)):
            # failures
            if self.results[i] == 'fail':
                outlier_times.append(self.times[i])
                outlier_trials.append(i)
            # successes, but with strange times
            elif self.times[i] > 40 or self.times[i]<15:
                strange_times.append(self.times[i])
                strange_trials.append(i)
            # normal success
            else:
                normal_times.append(self.times[i])
                normal_trials.append(i)
                
        fig,ax = plt.subplots(1)
        ax.scatter(normal_trials, normal_times, c='green', label = 'Success - Normal') # plot normal success times
        ax.scatter(strange_trials, strange_times, c = 'red', label = 'Success - Abnormal') # plot strange successes times
        ax.scatter(outlier_trials, outlier_times, c = 'red', marker = '^', label = 'Failure') # plot failure times
        ax.scatter(106, 10, c = 'white') # makes space to display average time
        ax.plot(np.linspace(self.avg_time, self.avg_time, 100), c= 'blue', label = 'Average Time') # plot average time
        ax.text(101, self.avg_time-.4,self.avg_time) # display average time
        
        ax.set_xticklabels([]) # remove x-axis labels
        plt.title('Approach Times')
        plt.ylabel('Seconds')
        ax.legend(loc="upper left")
        plt.savefig('/root/data/{}/approach_times.png'.format(self.name))
        plt.show()
    
    # copied (modified) from https://matplotlib.org/stable/gallery/lines_bars_and_markers/scatter_hist.html#sphx-glr-gallery-lines-bars-and-markers-scatter-hist-py
    def angle_vis(self):
        # YZ PLANES FROM Vectirs
        y = []
        z = []
        for vector in self.vectors:
            y.append(vector[1])
            z.append(vector[2])
        
        # MAKE THE CIRCLE CENTER EMPTY
        cam_circle = plt.Circle((0, 0), 5, color='black')
        
        # definitions for the axes
        left, width = 0.1, 0.65
        bottom, height = 0.1, 0.65
        spacing = 0.005

        rect_scatter = [left, bottom, width, height]
        rect_histx = [left, bottom + height + spacing, width, 0.2]
        rect_histy = [left + width + spacing, bottom, 0.2, height]

        # start with a square Figure
        fig = plt.figure(figsize=(8, 8))

        ax = fig.add_axes(rect_scatter)
        # GET CAM MEASUREMENTS
        ax.add_patch(cam_circle)
        ax_histx = fig.add_axes(rect_histx, sharex=ax)
        ax_histy = fig.add_axes(rect_histy, sharey=ax)

        # use the previously defined function
        scatter_hist(y, z, ax, ax_histx, ax_histy)
        
        plt.show()

data = DataVis('/root/data/2022-08-23 23:29/stage1_3.csv')
data.angle_vis()