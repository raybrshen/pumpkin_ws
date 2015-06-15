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
    print "============ Dynamic hand gestures: Circle"
    roscpp_initialize(sys.argv)
    rospy.init_node('pumpkin_planning', anonymous=True)

    right_arm = MoveGroupCommander("right_arm")
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
    print right_arm.get_current_pose().pose

    wpose = geometry_msgs.msg.Pose()
    wpose.orientation.w = 0.570398010274
    wpose.orientation.x = -0.279748061111
    wpose.orientation.y = -0.719670670452
    wpose.orientation.z = -0.280109368412
    wpose.position.y = -0.0349149588619
    wpose.position.z = 1.69789257029
    wpose.position.x =  0.0117939632591
    right_arm.set_pose_target(wpose)
    plan1 = right_arm.plan()

    right_arm.execute(plan1)
    print "============ Waiting while RVIZ executes plan1..."
    rospy.sleep(5)
    waypoints = []
    waypoints.append(right_arm.get_current_pose().pose)
    print right_arm.get_current_pose().pose
    gain = 0.05;
    r = 0.05
    a = 0.0117939632591 # waypoints[0].position.x
    b = 1.64789257 # waypoints[0].position.z
    #waypoints[0].position.x = a + r
    

    points = 10
    for i in xrange(points):
        wpose = geometry_msgs.msg.Pose()
        wpose.orientation.w = waypoints[i-1].orientation.w 
        wpose.orientation.x = waypoints[i-1].orientation.x 
        wpose.orientation.y = waypoints[i-1].orientation.y 
        wpose.orientation.z = waypoints[i-1].orientation.z 
        wpose.position.y = waypoints[i-1].position.y 
        wpose.position.z = b 
        wpose.position.x = a - r

        waypoints.append(copy.deepcopy(wpose))

    points = 10
    for i in xrange(points):
        wpose = geometry_msgs.msg.Pose()
        wpose.orientation.w = waypoints[i-1].orientation.w 
        wpose.orientation.x = waypoints[i-1].orientation.x 
        wpose.orientation.y = waypoints[i-1].orientation.y 
        wpose.orientation.z = waypoints[i-1].orientation.z 
        wpose.position.y = waypoints[i-1].position.y 
        wpose.position.z = b - r
        wpose.position.x = a

        waypoints.append(copy.deepcopy(wpose))

    points = 10
    for i in xrange(points):
        wpose = geometry_msgs.msg.Pose()
        wpose.orientation.w = waypoints[i-1].orientation.w 
        wpose.orientation.x = waypoints[i-1].orientation.x 
        wpose.orientation.y = waypoints[i-1].orientation.y 
        wpose.orientation.z = waypoints[i-1].orientation.z 
        wpose.position.y = waypoints[i-1].position.y 
        wpose.position.z = b 
        wpose.position.x = a + r
        
        waypoints.append(copy.deepcopy(wpose))

    points = 10
    for i in xrange(points):
        wpose = geometry_msgs.msg.Pose()
        wpose.orientation.w = waypoints[i-1].orientation.w 
        wpose.orientation.x = waypoints[i-1].orientation.x 
        wpose.orientation.y = waypoints[i-1].orientation.y 
        wpose.orientation.z = waypoints[i-1].orientation.z 
        wpose.position.y = waypoints[i-1].position.y 
        wpose.position.z = b + r
        wpose.position.x = a
        
        waypoints.append(copy.deepcopy(wpose))

    
    (circle, fraction) = right_arm.compute_cartesian_path(
                                                         waypoints,   # waypoints to follow
                                                         0.01,        # eef_step
                                                         0.0)         # jump_threshold
        
    print "============ Waiting while RVIZ displays Circle..."
    rospy.sleep(5)
    right_arm.execute(circle)
    rospy.sleep(5)

    roscpp_shutdown()