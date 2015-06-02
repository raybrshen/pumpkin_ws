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
    print "============ Starting tutorial setup"
    roscpp_initialize(sys.argv)
    rospy.init_node('pumpkin_planning', anonymous=True)

    right_arm = MoveGroupCommander("right_arm")
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
    
    waypoints = []
    waypoints.append(right_arm.get_current_pose().pose)
    # print right_arm.get_current_pose().pose
    """
    Define center
    position: 
      x: -0.0255748513923
      y: 0.114841815377
      z: 1.227117687
    orientation: 
      x: 0.0228873370986
      y: -0.706807582674
      z: 0.0230085660007
      w: 0.706661033853
    """
    gain = 0.05;
    r = 0.3
    a = -0.0255748513923 # waypoints[0].position.x
    b = 1.227117687 # waypoints[0].position.z
    waypoints[0].position.x = a + r
    

    points = 30
    for i in xrange(points):
        wpose = geometry_msgs.msg.Pose()
        wpose.orientation.w = waypoints[i-1].orientation.w
        wpose.orientation.x = waypoints[i-1].orientation.x
        wpose.orientation.y = waypoints[i-1].orientation.y
        wpose.orientation.z = waypoints[i-1].orientation.z
        wpose.position.y = waypoints[i-1].position.y
        wpose.position.z = waypoints[i-1].position.z
        wpose.position.x = r*sin(r*pi-r*pi*i)
        print r*sin(r*pi-r*pi*i)
    
        right_arm.set_pose_target(wpose)
        right_arm.plan()
        right_arm.go(wait=True)
        rospy.spin()
        #wpose.position.x = waypoints[i-1].position.x + gain/2
        #wpose.position.z = waypoints[i-1].position.z + copysign(gain, sin(i))/2
        
        #waypoints.append(copy.deepcopy(wpose))

    """
    (plan, fraction) = right_arm.compute_cartesian_path(
                                                         waypoints,   # waypoints to follow
                                                         0.01,        # eef_step
                                                         0.0)         # jump_threshold
    """
    #print "============ Waiting while RVIZ displays plan3..."
    #rospy.sleep(5)
    #right_arm.execute(plan)
    #print "============ Waiting while RVIZ execute?..."
    #rospy.sleep(5)
    roscpp_shutdown()

