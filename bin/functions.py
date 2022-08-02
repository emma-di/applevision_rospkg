#!/usr/bin/env python2
#for testing random things

from math import acos, degrees, sqrt
from geometry_msgs.msg import PointStamped
import rospy
import applevision_motion
import tf
from tf.listener import TransformListener
import moveit_commander
import sys
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import RobotTrajectory, Grasp, PlaceLocation, Constraints
from sensor_msgs.msg import JointState
import rospy
import tf
from moveit_ros_planning_interface import _moveit_move_group_interface
from exception import MoveItCommanderException

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
    x = (len(results))-1
    return (str(round((successes/float(x))*100, 2)) + " percent")

# computes the magnitude of a vector    
def magnitude(vector):
    return float(sqrt(vector[0]**2+vector[1]**2+vector[2]**2))
    
# takes two vectors and determines the angle between them    
def angle_success(v1, v2):
    dotproduct = float(v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2])
    angle = degrees(acos(dotproduct/(magnitude(v1)*magnitude(v2))))
    print(angle)
    if angle <= 5:
        return True
    else:
        return False

# gets average of values in a list from a certain point to the end
def average_value(list, start):
    sum = 0
    count = 0
    for x in range (start +1, len(list)+1):
        sum = sum + x
        print(sum)
        count += 1
    return(sum/count)