
#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

import sys
import copy
import rospy
import time
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
import numpy as np
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_commander.conversions import pose_to_list
## END_SUB_TUTORIAL

class jointPoint:
	def __init__(self):
		self.pos = 0
		self.vec = 0
		self.acc = 0
		self.time = 0


def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class MoveGroupPythonInteface(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonInteface, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface',
                    anonymous=True)

    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the Panda
    ## arm so we set ``group_name = panda_arm``. If you are using a different robot,
    ## you should change this value to the name of your robot arm planning group.
    ## This interface can be used to plan and execute motions on the Panda:
    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)

    ## We create a `DisplayTrajectory`_ publisher which is used later to publish
    ## trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    #print "============ Reference frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    #print "============ End effector: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    #print ""
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def go_home(self):
      group = self.group
      joint_goal = group.get_current_joint_values()
      joint_goal[0] = 0
      joint_goal[1] = 0
      joint_goal[2] = 0
      joint_goal[3] = 0 #+ 0.643501110063
      joint_goal[4] = 0
      joint_goal[5] = 0
      group.go(joint_goal, wait=True)
      print ("finished")
      print (self.group.get_current_pose().pose)
      group.stop()
      current_joints = self.group.get_current_joint_values()
      return all_close(joint_goal, current_joints, 0.01)

  def go_to_pose(self, pose):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    #test1 = rospy.get_time()
    #print ("test1 = ")
    #print (test1)
    group = self.group

    pose_goal = geometry_msgs.msg.Pose()
    group.set_pose_target(pose)

    ## Now, we call the planner to compute the plan and execute it.
    group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    print ("reach")
    group.stop()
    #test2 = rospy.get_time()
    #print ("test2 = ")
    #print (test2)
    #print (test2 - test1)
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


  def fast_go_to_pose(self, px, py, pz, ox, oy, oz, ow, ratio):
    group = self.group

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = ox
    pose_goal.orientation.y = oy
    pose_goal.orientation.z = oz
    pose_goal.orientation.w = ow
    pose_goal.position.x = px
    pose_goal.position.y = py
    pose_goal.position.z = pz
    group.set_pose_target(pose_goal)
    group.set_goal_orientation_tolerance

    traj = group.plan()

    new_traj = RobotTrajectory()
    new_traj = traj

    n_joints = len(traj.joint_trajectory.joint_names)
    n_points = len(traj.joint_trajectory.points)

    total_time = traj.joint_trajectory.points[n_points - 1].time_from_start

    spd = ratio

    points = list(traj.joint_trajectory.points)

    for i in range(n_points):
        point = JointTrajectoryPoint()
        point.time_from_start = traj.joint_trajectory.points[i].time_from_start / spd
        point.velocities = list(traj.joint_trajectory.points[i].velocities)
        point.accelerations = list(traj.joint_trajectory.points[i].accelerations)
        point.positions = traj.joint_trajectory.points[i].positions

        for j in range(n_joints):
            point.velocities[j] = point.velocities[j] * spd
            point.accelerations[j] = point.accelerations[j] * spd * spd

        points[i] = point

    new_traj.joint_trajectory.points = points

    group.execute(new_traj, wait=True)

def main():
    pub = rospy.Publisher('/arm_controller/command',JointTrajectory, queue_size=10)
    robot = MoveGroupPythonInteface()
    robot.go_home()
    pose_goal = geometry_msgs.msg.Pose()
    pose1 = geometry_msgs.msg.Pose()
    pose2 = geometry_msgs.msg.Pose()
    pose3 = geometry_msgs.msg.Pose()
    pose1.orientation.x = 0
    pose1.orientation.y = 0
    pose1.orientation.z = 0
    pose1.orientation.w = 1
    pose1.position.x = 0.5
    pose1.position.y = 0.5
    pose1.position.z = 0.5

    pose2.orientation.x = 0
    pose2.orientation.y = 0
    pose2.orientation.z = 0
    pose2.orientation.w = 1
    pose2.position.x = 0.55
    pose2.position.y = 0.4
    pose2.position.z = 0.4

    pose3.orientation.x = 0
    pose3.orientation.y = 0
    pose3.orientation.z = 0
    pose3.orientation.w = 1
    pose3.position.x = -0.4
    pose3.position.y = 0.4
    pose3.position.z = 0.7

    #robot.go_to_pose(pose1)

    robot.group.set_pose_target(pose2)
    robot.group.set_goal_orientation_tolerance
    traj1 = robot.group.plan()

    jointGroupPoints1 = []
    jointGroupPoints2 = []
    for i in range(6):
        jointIPoint1 = jointPoint()
        jointIPoint2 = jointPoint()
        jointIPoint2.pos = math.pi / 4
        jointIPoint2.vec = 0
        jointIPoint2.acc = 0
        jointIPoint2.time = 1
        jointGroupPoints1.append(jointIPoint1)
        jointGroupPoints2.append(jointIPoint2)

    traj = trajGenerateByTwoPoints(jointGroupPoints1, jointGroupPoints2)
    traj1.joint_trajectory.points = traj.points
    pub.publish(traj1.joint_trajectory)
    print("message has been published")


def real_robot_test():
    pub = rospy.Publisher('/pos_joint_traj_controller/command',JointTrajectory, queue_size=10)
    robot = MoveGroupPythonInteface()
    #robot.go_home()
    pose_goal = geometry_msgs.msg.Pose()
    pose1 = geometry_msgs.msg.Pose()
    pose2 = geometry_msgs.msg.Pose()
    pose3 = geometry_msgs.msg.Pose()
    pose1.orientation.x = 0
    pose1.orientation.y = 0
    pose1.orientation.z = 0
    pose1.orientation.w = 1
    pose1.position.x = 0.4
    pose1.position.y = 0.4
    pose1.position.z = 0.4

    pose2.orientation.x = 0
    pose2.orientation.y = 0
    pose2.orientation.z = 0
    pose2.orientation.w = 1
    pose2.position.x = -0.4
    pose2.position.y = 0.4
    pose2.position.z = 0.4

    pose3.orientation.x = 0
    pose3.orientation.y = 0
    pose3.orientation.z = 0
    pose3.orientation.w = 1
    pose3.position.x = -0.4
    pose3.position.y = 0.4
    pose3.position.z = 0.7

    robot.go_to_pose(pose1)

    robot.group.set_pose_target(pose2)
    robot.group.set_goal_orientation_tolerance
    traj1 = robot.group.plan()
    time_to_finish = traj1.joint_trajectory.points[-1].time_from_start.nsecs/(1e9)
    

    robot.group.set_pose_target(pose3)
    robot.group.set_goal_orientation_tolerance
    traj2 = robot.group.plan()

    pub.publish(traj1.joint_trajectory)
    print("start moving")
    time.sleep(time_to_finish/2)

    pub.publish(traj2.joint_trajectory)

    robot.group.execute(traj2, wait=True)


    time.sleep(5)

    robot.group.set_pose_target(pose1)
    robot.group.set_goal_orientation_tolerance
    traj3 = robot.group.plan()
    pub.publish(traj3.joint_trajectory)


def trajGenerateByTwoPoints(jointGroupPointsStart, jointGroupPointsGoal, deltaT = 0.01):
	#initialize trajectory message
	traj = JointTrajectory()
	traj.header.stamp = rospy.Time.now()
	traj.header.frame_id = "world" #replace this

	jointNamesList = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
  'wrist_3_joint']
	traj.joint_names = jointNamesList

	#append point
	#calculate parameters for each joint
	paraList = []
	for jointPointStart, jointPointEnd in zip(jointGroupPointsStart, jointGroupPointsGoal):
		timeS = jointPointStart.time
		timeE = jointPointEnd.time
		A = np.mat([
			[1, timeS, math.pow(timeS, 2), math.pow(timeS, 3), math.pow(timeS, 4), math.pow(timeS, 5)],
			[0, 1, 2*timeS, 3*math.pow(timeS, 2), 4*math.pow(timeS, 3), 5*math.pow(timeS, 4)],
			[0, 0, 2, 6*timeS, 12*math.pow(timeS,2), 20*math.pow(timeS,3)],
			[1, timeE, math.pow(timeE, 2), math.pow(timeE, 3), math.pow(timeE, 4), math.pow(timeE, 5)],
			[0, 1, 2*timeE, 3*math.pow(timeE, 2), 4*math.pow(timeE, 3), 5*math.pow(timeE, 4)],
			[0, 0, 2, 6*timeE, 12*math.pow(timeE,2), 20*math.pow(timeE,3)]
			])
		b = np.array([jointPointStart.pos, jointPointStart.vec, jointPointStart.acc, jointPointEnd.pos, jointPointEnd.vec, jointPointEnd.acc])
		x = np.linalg.solve(A, b)
		paraList.append(x)


	#append points
	for i in range(int(math.ceil((timeE - timeS) / deltaT))):
		timeI = timeS + i*deltaT
		pointI = JointTrajectoryPoint()
		for jointNo in range(len(jointNamesList)):
			posI = np.dot(
				np.array([1, timeI, math.pow(timeI, 2), math.pow(timeI, 3), math.pow(timeI, 4), math.pow(timeI, 5)]),
				np.array(paraList[jointNo])
				)
			vecI = np.dot(
				np.array([0, 1, 2*timeI, 3*math.pow(timeI, 2), 4*math.pow(timeI, 3), 5*math.pow(timeI, 4)]),
				np.array(paraList[jointNo])
				)
			accI = np.dot(
				[0, 0, 2, 6*timeI, 12*math.pow(timeI,2), 20*math.pow(timeI,3)],
				np.array(paraList[jointNo])
				)
			pointI.positions.append(posI)
			pointI.velocities.append(vecI)
			pointI.accelerations.append(accI)

		pointI.time_from_start = rospy.Duration.from_sec(timeI)
		traj.points.append(pointI)

	lastPoint = JointTrajectoryPoint()
	for jointNo in range(len(jointNamesList)):
		lastPoint.positions.append(jointGroupPointsGoal[jointNo].pos)
		lastPoint.velocities.append(jointGroupPointsGoal[jointNo].vec)
		lastPoint.accelerations.append(jointGroupPointsGoal[jointNo].acc)
		lastPoint.time_from_start = rospy.Duration.from_sec(jointGroupPointsGoal[jointNo].time)

	traj.points.append(lastPoint)

	return traj




if __name__ == '__main__':
    #real_robot_test()
    main()
