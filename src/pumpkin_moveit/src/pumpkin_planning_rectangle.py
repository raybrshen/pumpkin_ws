#!/usr/bin/env python
# coding: utf-8
import sys
import rospy
import copy
import geometry_msgs.msg
import moveit_msgs.msg

from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown

from math import sin, copysign, sqrt, pi
   
if __name__ == '__main__':
    print "============ Dynamic hand gestures"
    roscpp_initialize(sys.argv)
    rospy.init_node('pumpkin_planning', anonymous=True)

    right_arm = MoveGroupCommander("right_arm")
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
    print right_arm.get_current_pose().pose

    wpose = geometry_msgs.msg.Pose()
    wpose.orientation.w = 0.637544120474
    wpose.orientation.x = -0.250890131255
    wpose.orientation.y = -0.661106583226
    wpose.orientation.z = -0.305826294003
    wpose.position.y = 0.060301362088
    wpose.position.z = 1.53243619859
    wpose.position.x = 0.129928650473
    right_arm.set_pose_target(wpose)
    plan1 = right_arm.plan()

    print "============ Waiting while RVIZ displays plan1..."
    rospy.sleep(5)
    right_arm.execute(plan1)
    print "============ Waiting while RVIZ executes plan1..."
    rospy.sleep(5)
    waypoints = []
    waypoints.append(right_arm.get_current_pose().pose)
    print right_arm.get_current_pose().pose
    gain = 0.05;
    r = 0.3
    a = -0.0255748513923 # waypoints[0].position.x
    b = 1.227117687 # waypoints[0].position.z
    #waypoints[0].position.x = a + r
    

    points = 10
    for i in xrange(points):
        wpose = geometry_msgs.msg.Pose()
        wpose.orientation.w = waypoints[i-1].orientation.w 
        wpose.orientation.x = waypoints[i-1].orientation.x 
        wpose.orientation.y = waypoints[i-1].orientation.y 
        wpose.orientation.z = waypoints[i-1].orientation.z 
        wpose.position.y = waypoints[i-1].position.y 
        wpose.position.z = waypoints[i-1].position.z 
        wpose.position.x = waypoints[i-1].position.x - 0.025
        """
        right_arm.set_pose_target(wpose)
        right_arm.plan()
        right_arm.go(wait=True)
        rospy.spin()
        """
        #wpose.position.x = waypoints[i-1].position.x + gain/2
        #wpose.position.z = waypoints[i-1].position.z + copysign(gain, sin(i))/2
        
        waypoints.append(copy.deepcopy(wpose))

    
    (plan3, fraction) = right_arm.compute_cartesian_path(
                                                         waypoints,   # waypoints to follow
                                                         0.01,        # eef_step
                                                         0.0)         # jump_threshold
    
    print "============ Waiting while RVIZ displays plan3..."
    rospy.sleep(5)
    right_arm.execute(plan3)
    rospy.sleep(5)
    
    points = 10
    for i in xrange(points):
        wpose = geometry_msgs.msg.Pose()
        wpose.orientation.w = waypoints[i-1].orientation.w 
        wpose.orientation.x = waypoints[i-1].orientation.x 
        wpose.orientation.y = waypoints[i-1].orientation.y 
        wpose.orientation.z = waypoints[i-1].orientation.z 
        wpose.position.y = waypoints[i-1].position.y 
        wpose.position.z = waypoints[i-1].position.z - 0.25
        wpose.position.x = waypoints[i-1].position.x 
        """
        right_arm.set_pose_target(wpose)
        right_arm.plan()
        right_arm.go(wait=True)
        rospy.spin()
        """
        #wpose.position.x = waypoints[i-1].position.x + gain/2
        #wpose.position.z = waypoints[i-1].position.z + copysign(gain, sin(i))/2
        
        waypoints.append(copy.deepcopy(wpose))

    
    (plan4, fraction) = right_arm.compute_cartesian_path(
                                                         waypoints,   # waypoints to follow
                                                         0.01,        # eef_step
                                                         0.0)         # jump_threshold
    
    print "============ Waiting while RVIZ displays plan4..."
    rospy.sleep(5)
    right_arm.execute(plan4)
    rospy.sleep(5)
    
    points = 10
    for i in xrange(points):
        wpose = geometry_msgs.msg.Pose()
        wpose.orientation.w = waypoints[i-1].orientation.w 
        wpose.orientation.x = waypoints[i-1].orientation.x 
        wpose.orientation.y = waypoints[i-1].orientation.y 
        wpose.orientation.z = waypoints[i-1].orientation.z 
        wpose.position.y = waypoints[i-1].position.y 
        wpose.position.z = waypoints[i-1].position.z 
        wpose.position.x = waypoints[i-1].position.x + 0.025
        
        waypoints.append(copy.deepcopy(wpose))
    (plan5, fraction) = right_arm.compute_cartesian_path(
                                                         waypoints,   # waypoints to follow
                                                         0.01,        # eef_step
                                                         0.0)         # jump_threshold
    
    print "============ Waiting while RVIZ displays plan5..."
    rospy.sleep(5)
    right_arm.execute(plan5)
    rospy.sleep(5)   
    points = 10
    for i in xrange(points):
        wpose = geometry_msgs.msg.Pose()
        wpose.orientation.w = waypoints[i-1].orientation.w 
        wpose.orientation.x = waypoints[i-1].orientation.x 
        wpose.orientation.y = waypoints[i-1].orientation.y 
        wpose.orientation.z = waypoints[i-1].orientation.z 
        wpose.position.y = waypoints[i-1].position.y 
        wpose.position.z = waypoints[i-1].position.z + 0.25
        wpose.position.x = waypoints[i-1].position.x 
        
        waypoints.append(copy.deepcopy(wpose))

    
    (plan6, fraction) = right_arm.compute_cartesian_path(
                                                         waypoints,   # waypoints to follow
                                                         0.01,        # eef_step
                                                         0.0)         # jump_threshold
    
    print "============ Waiting while RVIZ displays plan6..."
    rospy.sleep(5)
    right_arm.execute(plan6)
    #print "============ Waiting while RVIZ execute?..."
    #rospy.sleep(5)

    roscpp_shutdown()