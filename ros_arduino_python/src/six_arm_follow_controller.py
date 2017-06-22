# -*- coding: UTF-8 -*-
#!/usr/bin/env python
# Copyright (c) 2017-2018 williamar. 
# All right reserved.

import rospy, actionlib

from control_msgs.msg import FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectory
from diagnostic_msgs.msg import *
from math import pi as PI, degrees, radians
from joints import Jloint
class FollowController:
#此类是驱动的核心代码
    def __init__(self, name):
        self.name = name

        rospy.init_node(self.name)

        #初始化化ros note，设置节点名称，刷新频率为50hz
        # rates
        self.rate = 50.0
        #初始化机械臂的关节，并把关节放入joints列表中，方便后续操作
        # Arm jonits
        self.arm_base_to_arm_round_joint_stevo0=Joint('body_to_left_shoulder_joint',3,1.5797,-1.5707,130,0,False)
        self.shoulder_2_to_arm_joint_stevo1=Joint('shoulder_stevo_lift_to_axis',4,1.5707,-0.1899,115,45,False)
        self.big_arm_round_to_joint_stevo2=Joint('big_arm_up_to_axis',5,2.5891,1,100,20,False)
        self.arm_joint_stevo2_to_arm_joint_stevo3=Joint('small_arm_up_to_axis',6,1.5707,-1.5707,130,0,False)
        self.wrist_to_arm_joint_stevo4=Joint('wrist_run_stevo_to_axis',7,1.5707,-1.5707,130,0,False)
        self.arm_joint_stevo4_to_arm_joint_stevo5=Joint('arm_joint_stevo4_to_arm_joint_stevo5',8,1.5707,-1.5707,130,0,True)



        self.joints=[self.arm_base_to_arm_round_joint_stevo0,
        self.shoulder_2_to_arm_joint_stevo1,
        self.big_arm_round_to_joint_stevo2,
        self.arm_joint_stevo2_to_arm_joint_stevo3,
        self.wrist_to_arm_joint_stevo4,
        self.arm_joint_stevo4_to_arm_joint_stevo5]


        # action server
        self.server = actionlib.SimpleActionServer('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction, execute_cb=self.actionCb, auto_start=False)
        self.server.start()
        rospy.spin()
        rospy.loginfo("Started FollowController")


    def actionCb(self, goal):
        print("****************actionCb*********************")
        rospy.loginfo(self.name + ": Action goal recieved.")
        traj = goal.trajectory
        rospy.logerr("msg11111")
        if set(self.joints) != set(traj.joint_names):
            for j in self.joints:
                if j.name not in traj.joint_names:
                    msg = "Trajectory joint names does not match action controlled joints." + str(traj.joint_names)
                    rospy.logerr("msg22222")
                    rospy.logerr(msg)
                    self.server.set_aborted(text=msg)
                    return
            rospy.logwarn("Extra joints in trajectory")

        if not traj.points:
            msg = "Trajectory empy."
            rospy.logerr(msg)
            self.server.set_aborted(text=msg)
            return

        try:
            indexes = [traj.joint_names.index(joint.name) for joint in self.joints]
        except ValueError as val:
            msg = "Trajectory invalid."
            rospy.logerr(msg)
            self.server.set_aborted(text=msg)
            return

        if self.executeTrajectory(traj):   
            self.server.set_succeeded()
        else:
            self.server.set_aborted(text="Execution failed.")

        rospy.loginfo(self.name + ": Done.")     

    def executeTrajectory(self, traj):
        rospy.loginfo("Executing trajectory")
        rospy.logdebug(traj)
        # carry out trajectory
        try:
            indexes = [traj.joint_names.index(joint.name) for joint in self.joints]
        except ValueError as val:
            rospy.logerr("Invalid joint in trajectory.")
            return False

        # get starting timestamp, MoveIt uses 0, need to fill in
        start = traj.header.stamp
        if start.secs == 0 and start.nsecs == 0:
            start = rospy.Time.now()

        r = rospy.Rate(self.rate)
    for point in traj.points:            
            desired = [ point.positions[k] for k in indexes ]
            for i in indexes:
                #self.joints[i].position=desired[i]
                self.joints[i].setCurrentPosition(desired[i])

            while rospy.Time.now() + rospy.Duration(0.01) < start:
                rospy.sleep(0.01)
    return True


if __name__=='__main__':
    try:
        rospy.loginfo("start followController...")
        FollowController('follow_Controller')
    except rospy.ROSInterruptException:
        rospy.loginfo('Failed to start followController...')
