#!/usr/bin/env python2
#for testing random things

from math import sqrt
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