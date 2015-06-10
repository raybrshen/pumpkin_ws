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

    group_variable_values = right_arm.get_current_joint_values()
    print "============ Joint values: ", group_variable_values

    group_variable_values[0] = -0.357569385437786
    group_variable_values[1] = -0.6320268016619928
    group_variable_values[2] = 0.24177580736353846
    group_variable_values[3] = 1.586101553004471
    group_variable_values[4] = -0.5805943181752088
    group_variable_values[5] = -1.1821499952996368
    right_arm.set_joint_value_target(group_variable_values)

    right_arm.go(wait=True)

    print "============ Waiting while RVIZ displays Up..."

    group_variable_values[0] = 1.0
    right_arm.set_joint_value_target(group_variable_values)

    right_arm.go(wait=True)

    group_variable_values = right_arm.get_current_joint_values()

    for i in xrange(6):
        group_variable_values[i] = 0
    right_arm.set_joint_value_target(group_variable_values)

    right_arm.go(wait=True)

    group_variable_values[0] = 0.9999918358930386
    group_variable_values[1] = -0.6320841964382067
    group_variable_values[2] = 0.24174252879256236
    group_variable_values[3] = 1.5860776440417892
    group_variable_values[4] = -0.5806558589477884
    group_variable_values[5] = -1.1822275779320612
    right_arm.set_joint_value_target(group_variable_values)

    right_arm.go(wait=True)

    print "============ Waiting while RVIZ displays Down..."

    group_variable_values[0] = -0.357569385437786
    group_variable_values[1] = -0.6320268016619928
    group_variable_values[2] = 0.24177580736353846
    group_variable_values[3] = 1.586101553004471
    group_variable_values[4] = -0.5805943181752088
    group_variable_values[5] = -1.1821499952996368
    right_arm.set_joint_value_target(group_variable_values)

    right_arm.go(wait=True)

    for i in xrange(6):
        group_variable_values[i] = 0
    right_arm.set_joint_value_target(group_variable_values)
    right_arm.go(wait=True)

    group_variable_values[0] = -0.357569385437786
    group_variable_values[1] = -0.6320268016619928
    group_variable_values[2] = 0.24177580736353846
    group_variable_values[3] = 1.586101553004471
    group_variable_values[4] = -0.5805943181752088
    group_variable_values[5] = -1.1821499952996368
    right_arm.set_joint_value_target(group_variable_values)

    right_arm.go(wait=True)

    print "============ Waiting while RVIZ displays Left..."

    group_variable_values[0] = -0.23537138466827642
    group_variable_values[1] = -0.9560656584079451
    group_variable_values[2] = -1.0567864214546296
    group_variable_values[3] = 1.0063311554535113
    group_variable_values[4] = -0.6841497505013784
    group_variable_values[5] = -1.1821740994324195
    right_arm.set_joint_value_target(group_variable_values)

    right_arm.go(wait=True)

    print "============ Waiting while RVIZ displays Right..."

    for i in xrange(6):
        group_variable_values[i] = 0
    right_arm.set_joint_value_target(group_variable_values)
    right_arm.go(wait=True)

    print "============ Waiting while RVIZ displays Come..."

    group_variable_values[0] = -0.27140437982040627
    group_variable_values[1] = -0.7922322322014895
    group_variable_values[2] = 1.6564152017121085
    group_variable_values[3] = 0.605540013918081
    group_variable_values[4] = -1.2851244289045503
    group_variable_values[5] = -1.1820720019244342
    right_arm.set_joint_value_target(group_variable_values)
    right_arm.go(wait=True)

    group_variable_values[0] = -0.42590805556554895
    group_variable_values[1] = -0.8207508578135979
    group_variable_values[2] = 0.37069364500123253
    group_variable_values[3] = 1.5338220570582606
    group_variable_values[4] = -0.7310422288227206
    group_variable_values[5] = -0.9623657998261584
    right_arm.set_joint_value_target(group_variable_values)
    right_arm.go(wait=True)

    group_variable_values[0] = 0.2275545602635755
    group_variable_values[1] = -0.6994757899464749
    group_variable_values[2] = -0.2507500485665307
    group_variable_values[3] = 1.5361284439917693
    group_variable_values[4] = -0.6764466998809395
    group_variable_values[5] = -1.8415444610927885
    right_arm.set_joint_value_target(group_variable_values)
    right_arm.go(wait=True)

    print "============ Waiting while RVIZ displays Go..."

    group_variable_values[0] = -0.42590805556554895
    group_variable_values[1] = -0.8207508578135979
    group_variable_values[2] = 0.37069364500123253
    group_variable_values[3] = 1.5338220570582606
    group_variable_values[4] = -0.7310422288227206
    group_variable_values[5] = -0.9623657998261584
    right_arm.set_joint_value_target(group_variable_values)
    right_arm.go(wait=True)

    group_variable_values[0] = -0.27140437982040627
    group_variable_values[1] = -0.7922322322014895
    group_variable_values[2] = 1.6564152017121085
    group_variable_values[3] = 0.605540013918081
    group_variable_values[4] = -1.2851244289045503
    group_variable_values[5] = -1.1820720019244342
    right_arm.set_joint_value_target(group_variable_values)
    right_arm.go(wait=True)

    print "============ Waiting while RVIZ displays Bye..."
    group_variable_values[0] = 0.25306013949285366
    group_variable_values[1] = -0.06086513735285551
    group_variable_values[2] = -0.1716742770211111
    group_variable_values[3] = 1.2479639578410286
    group_variable_values[4] = -0.04529914437657169
    group_variable_values[5] = -1.43227514664159
    right_arm.set_joint_value_target(group_variable_values)
    right_arm.go(wait=True)

    group_variable_values[0] = 0.3371402876197415
    group_variable_values[1] = -0.7622545280824188
    group_variable_values[2] = -0.2955701916375695
    group_variable_values[3] = 1.4574038743443292
    group_variable_values[4] = -0.7077652736412722
    group_variable_values[5] = -1.8977835953194326
    right_arm.set_joint_value_target(group_variable_values)
    right_arm.go(wait=True)

    group_variable_values[0] = 0.8376639429681074
    group_variable_values[1] = -0.9918830444159654
    group_variable_values[2] = -0.2955701916375695
    group_variable_values[3] = -1.4171103128802764
    group_variable_values[4] =  0.8785482029614121
    group_variable_values[5] = -0.30272420279455187
    right_arm.set_joint_value_target(group_variable_values)
    right_arm.go(wait=True)

    group_variable_values[0] = 0.3371402876197415
    group_variable_values[1] = -0.7622545280824188
    group_variable_values[2] = -0.2955701916375695
    group_variable_values[3] = 1.4574038743443292
    group_variable_values[4] = -0.7077652736412722
    group_variable_values[5] = -1.8977835953194326
    right_arm.set_joint_value_target(group_variable_values)
    right_arm.go(wait=True)

    group_variable_values[0] = 0.2885396759275396
    group_variable_values[1] = 0.14501022505945102
    group_variable_values[2] = -0.3895907892080015
    group_variable_values[3] = 0.6637670541599332
    group_variable_values[4] = 0.24255763591509477
    group_variable_values[5] = -0.8163938051951068
    right_arm.set_joint_value_target(group_variable_values)
    right_arm.go(wait=True)

    group_variable_values[0] = 0.4251283249832232
    group_variable_values[1] = -0.3596100418457812
    group_variable_values[2] = -0.5046395989165803
    group_variable_values[3] = 1.366728753267692
    group_variable_values[4] = -0.22584200551718175
    group_variable_values[5] = -1.818044521418347
    right_arm.set_joint_value_target(group_variable_values)
    right_arm.go(wait=True)

    print "============ Waiting while RVIZ displays Rectangle..."
    group_variable_values[0] = 0.26406732642545633
    group_variable_values[1] = 0.22910732531589367
    group_variable_values[2] = -0.6129325291020364
    group_variable_values[3] = 0.8435855585824819
    group_variable_values[4] = 0.3111993084421887
    group_variable_values[5] = -0.9011691714085972
    right_arm.set_joint_value_target(group_variable_values)
    right_arm.go(wait=True)

    group_variable_values[0] = 0.16078242293673548
    group_variable_values[1] = -0.19601699834600583
    group_variable_values[2] = -0.34203339774622604
    group_variable_values[3] = 1.4916830161440273
    group_variable_values[4] = -0.1729057720909184
    group_variable_values[5] = -1.665281825951373
    right_arm.set_joint_value_target(group_variable_values)
    right_arm.go(wait=True)

    group_variable_values[0] = 0.6675686089699095
    group_variable_values[1] = -1.0203045107902358
    group_variable_values[2] =  0.025086074715929895
    group_variable_values[3] = 1.041252252310127
    group_variable_values[4] =  -1.0049084472088514
    group_variable_values[5] = -1.691788307225158
    right_arm.set_joint_value_target(group_variable_values)
    right_arm.go(wait=True)

    group_variable_values[0] = -0.270677562484728
    group_variable_values[1] = -0.8515995139221347
    group_variable_values[2] =  -0.34979566111198535
    group_variable_values[3] = 1.521619885163895
    group_variable_values[4] =  -0.9166797265028328
    group_variable_values[5] = -1.599384852084805
    right_arm.set_joint_value_target(group_variable_values)
    right_arm.go(wait=True)

    group_variable_values[0] = -0.3764791205315819
    group_variable_values[1] = -0.03697163040647115
    group_variable_values[2] = -0.06249320879809678
    group_variable_values[3] = 1.203433321430528
    group_variable_values[4] =  -0.13800960047536145
    group_variable_values[5] = -0.9594665790267604
    right_arm.set_joint_value_target(group_variable_values)
    right_arm.go(wait=True)

    group_variable_values[0] = 1.0702924527656503
    group_variable_values[1] = 0.17542257528403507
    group_variable_values[2] = -1.5978270458672195
    group_variable_values[3] = 0.8770213961532071
    group_variable_values[4] =  0.6757950278427888
    group_variable_values[5] = -1.122426562596535
    right_arm.set_joint_value_target(group_variable_values)
    right_arm.go(wait=True)

    print "============ Waiting while RVIZ displays Triangle..."
    group_variable_values[0] = 0.9240110305690618
    group_variable_values[1] = -0.09272799668278685
    group_variable_values[2] = -1.11179978037871
    group_variable_values[3] = 1.0860766627811735
    group_variable_values[4] = 0.353203157245395
    group_variable_values[5] = -1.6353720001060041
    right_arm.set_joint_value_target(group_variable_values)
    right_arm.go(wait=True)

    group_variable_values[0] = 0.293240626454948
    group_variable_values[1] = -0.07913408933457208
    group_variable_values[2] = -0.8643005458360863
    group_variable_values[3] = 1.6138518956585908
    group_variable_values[4] = 0.03767269195542487
    group_variable_values[5] = -1.837524350155922
    right_arm.set_joint_value_target(group_variable_values)
    right_arm.go(wait=True)

    group_variable_values[0] = -0.16025919408469574
    group_variable_values[1] = -0.8763385307701629
    group_variable_values[2] = -0.7046571823914376
    group_variable_values[3] = 1.3613599249757264
    group_variable_values[4] = -0.8244634986709861
    group_variable_values[5] = -1.8426642621144569
    right_arm.set_joint_value_target(group_variable_values)
    right_arm.go(wait=True)

    group_variable_values[0] = -0.29728787446794586
    group_variable_values[1] = 0.18261629386447173
    group_variable_values[2] = -0.4245594068035203
    group_variable_values[3] = 1.1639597317916655
    group_variable_values[4] =  0.008340891570893887
    group_variable_values[5] = -0.9304390465975989
    right_arm.set_joint_value_target(group_variable_values)
    right_arm.go(wait=True)

    group_variable_values[0] = 0.6535510975381349
    group_variable_values[1] = 0.29252314125404993
    group_variable_values[2] = -1.2859918167150972
    group_variable_values[3] = 1.3118078783040723
    group_variable_values[4] =  0.4204660893667271
    group_variable_values[5] =  -1.3107848285195551
    right_arm.set_joint_value_target(group_variable_values)
    right_arm.go(wait=True)

    group_variable_values[0] = 0.5167700981810514
    group_variable_values[1] = -0.3922113218417668
    group_variable_values[2] = -0.5375371057249607
    group_variable_values[3] = 1.3516857583486068
    group_variable_values[4] = -0.22063183226057737
    group_variable_values[5] = -1.8880175785316815
    right_arm.set_joint_value_target(group_variable_values)
    right_arm.go(wait=True)

    print "============ Waiting while RVIZ displays Wait..."
    group_variable_values[0] = 0.4160639644028851
    group_variable_values[1] = -0.09133113224124285
    group_variable_values[2] =  -0.48756948551232604
    group_variable_values[3] = 1.3055916115441428
    group_variable_values[4] =  -0.0008423995315249818
    group_variable_values[5] = -1.6193309487434702
    right_arm.set_joint_value_target(group_variable_values)
    right_arm.go(wait=True)
    """
    waypoints = []
    waypoints.append(right_arm.get_current_pose().pose)
    # print right_arm.get_current_pose().pose
    
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

    
    (plan3, fraction) = right_arm.compute_cartesian_path(
                                                         waypoints,   # waypoints to follow
                                                         0.01,        # eef_step
                                                         0.0)         # jump_threshold
    
    print "============ Waiting while RVIZ displays plan3..."
    rospy.sleep(5)
    #right_arm.execute(plan)
    #print "============ Waiting while RVIZ execute?..."
    #rospy.sleep(5)
    """
    roscpp_shutdown()

