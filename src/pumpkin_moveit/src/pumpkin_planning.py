#!/usr/bin/env python

import sys
import rospy
import copy
import moveit_commander
import geometry_msgs.msg
import moveit_msgs.msg

if __name__ == '__main__':
    print "============ Starting tutorial setup"
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('pumpkin_planning', anonymous=True)

    group = moveit_commander.MoveGroupCommander("right_arm")
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10)
    
    waypoints = []
    waypoints.append(group.get_current_pose().pose)
    # first orient gripper and move forward (+x)
    print group.get_current_pose().pose
    for i in xrange(10):
        wpose = geometry_msgs.msg.Pose()
        wpose.orientation.w = waypoints[i-1].orientation.w
        wpose.orientation.x = waypoints[i-1].orientation.x
        wpose.orientation.y = waypoints[i-1].orientation.y
        wpose.orientation.z = waypoints[i-1].orientation.z
        wpose.position.x = waypoints[i-1].position.x - 0.1
        wpose.position.y = waypoints[i-1].position.y
        wpose.position.z = waypoints[i-1].position.z
        print copy.deepcopy(wpose)
        waypoints.append(copy.deepcopy(wpose))

    (plan3, fraction) = group.compute_cartesian_path(
                                                     waypoints,   # waypoints to follow
                                                     0.01,        # eef_step
                                                     0.0)         # jump_threshold

    print "============ Waiting while RVIZ displays plan3..."
    rospy.sleep(5)


