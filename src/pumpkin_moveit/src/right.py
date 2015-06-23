#!/usr/bin/env python
# coding: utf-8
import sys
import rospy
import copy
import geometry_msgs.msg

from moveit_commander import MoveGroupCommander
from moveit_commander import roscpp_initialize, roscpp_shutdown

from math import sin, copysign

if __name__ == '__main__':
    print "--- Straight line gesture ---"
#     roscpp_initialize(sys.argv)
    rospy.init_node('straight_line', anonymous=True)

    right_arm = MoveGroupCommander("right_arm")
    start_pose = geometry_msgs.msg.Pose()
    start_pose.position.x = -0.0988490064784
    start_pose.position.y = 0.272349904278
    start_pose.position.z = 1.18864499931
    start_pose.orientation.x = 0.393751611087
    start_pose.orientation.y = 0.918424640162
    start_pose.orientation.z = -0.0150455838492
    start_pose.orientation.w = 0.0350639347048
#     start_pose.orientation.w = 0
#     start_pose.orientation.x = 0
#     start_pose.orientation.y = 1
#     start_pose.orientation.z = 0
#     start_pose.position.y = 0.0256415233819
#     start_pose.position.z = 1.25871460368
#     start_pose.position.x = 0.243500142238
    right_arm.set_pose_target(start_pose)
    plan_start = right_arm.plan()
    print "============ Waiting while RVIZ displays plan_start..."
    rospy.sleep(5)
    right_arm.execute(plan_start)
    print "============ Waiting while RVIZ executes plan_start..."
    rospy.sleep(5)
     
    waypoints = []
    waypoints.append(right_arm.get_current_pose().pose)
 
    gain = 0.2
    points = 5
    for i in xrange(points):
        wpose = geometry_msgs.msg.Pose()
        wpose.orientation.w = waypoints[i-1].orientation.w
        wpose.orientation.x = waypoints[i-1].orientation.x
        wpose.orientation.y = waypoints[i-1].orientation.y
        wpose.orientation.z = waypoints[i-1].orientation.z
        wpose.position.y = waypoints[i-1].position.y - 0.0737309
        wpose.position.z = waypoints[i-1].position.z + 0.02544397
        wpose.position.x = waypoints[i-1].position.x + 0.1991539
 
        waypoints.append(copy.deepcopy(wpose))
 
     
    (plan_waypoints, fraction) = right_arm.compute_cartesian_path(
                                                             waypoints,   # waypoints to follow
                                                             0.01,        # eef_step
                                                             0.0)         # jump_threshold
    print fraction*100, "% planned"
     
    print "============ Waiting while RVIZ displays plan3..."
    rospy.sleep(5)    
    right_arm.execute(plan_waypoints)
    print "============ Waiting while RVIZ execute..."
    rospy.sleep(5)
     
