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
    print "============ Dynamic hand gestures: Triangle"
    roscpp_initialize(sys.argv)
    rospy.init_node('pumpkin_planning', anonymous=True)

    right_arm = MoveGroupCommander("right_arm")
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
    print right_arm.get_current_pose().pose

    start_pose = geometry_msgs.msg.Pose()
    start_pose.position.x = -0.0983760947206
    start_pose.position.y = 0.0227414167935
    start_pose.position.z = 1.26769963627
    start_pose.orientation.x = 0.5
    start_pose.orientation.y = 0.5
    start_pose.orientation.z = 0.5
    start_pose.orientation.w = 0.5
    right_arm.set_pose_target(start_pose)
    plan_start = right_arm.plan()

    right_arm.execute(plan_start)
    print "============ Waiting while RVIZ executes plan_start..."
    rospy.sleep(5)
    waypoints = []
    waypoints.append(right_arm.get_current_pose().pose)
    print right_arm.get_current_pose().pose
    gain = 0.05
    points = 20
    step = points/4

    for i in xrange(step):
        start_pose = geometry_msgs.msg.Pose()
        start_pose.orientation = waypoints[i-1].orientation
        start_pose.position.y = waypoints[i-1].position.y 
        start_pose.position.z = waypoints[i-1].position.z + gain
        start_pose.position.x = waypoints[i-1].position.x + gain

        waypoints.append(copy.deepcopy(start_pose))
  
    (plan_waypoints, fraction) = right_arm.compute_cartesian_path(
                                                             waypoints,   # waypoints to follow
                                                             0.01,        # eef_step
                                                             0.0)         # jump_threshold
    print fraction*100, "% planned"
    print "============ Waiting while RVIZ displays plan_waypoints..."
    right_arm.execute(plan_waypoints)
    rospy.sleep(5)
    
    waypoints = []
    waypoints.append(right_arm.get_current_pose().pose)   

    for i in xrange(step):
        start_pose = geometry_msgs.msg.Pose()
        start_pose.orientation = waypoints[i-1].orientation
        start_pose.position.y = waypoints[i-1].position.y 
        start_pose.position.z = waypoints[i-1].position.z - gain
        start_pose.position.x = waypoints[i-1].position.x + gain
        waypoints.append(copy.deepcopy(start_pose))
 
    (plan_waypoints, fraction) = right_arm.compute_cartesian_path(
                                                             waypoints,   # waypoints to follow
                                                             0.01,        # eef_step
                                                             0.0)         # jump_threshold
    print fraction*100, "% planned"
    print "============ Waiting while RVIZ displays plan_waypoints..."
    right_arm.execute(plan_waypoints)
    rospy.sleep(5)
    
    waypoints = []
    waypoints.append(right_arm.get_current_pose().pose)
    points = 10
    for i in xrange(points):
        start_pose = geometry_msgs.msg.Pose()
        start_pose.orientation = waypoints[i-1].orientation
        start_pose.position.y = waypoints[i-1].position.y 
        start_pose.position.z = waypoints[i-1].position.z
        start_pose.position.x = waypoints[i-1].position.x - gain
        waypoints.append(copy.deepcopy(start_pose))
    
 
    (plan_waypoints, fraction) = right_arm.compute_cartesian_path(
                                                             waypoints,   # waypoints to follow
                                                             0.01,        # eef_step
                                                             0.0)         # jump_threshold
    print fraction*100, "% planned"
    print "============ Waiting while RVIZ displays plan_waypoints..."
    right_arm.execute(plan_waypoints)
    rospy.sleep(5)
