import rospy, math, time
import numpy as np
import matplotlib.pyplot as plt
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class jointPoint:
	def __init__(self):
		self.pos = 0
		self.vec = 0
		self.acc = 0
		self.time = 0
		

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
	rospy.init_node('quadrillion_test', anonymous=True)
	pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
	jointGroupPoints1 = []
	jointGroupPoints2 = []
	for i in range(6):
		jointIPoint1 = jointPoint()
		jointIPoint2 = jointPoint()
		jointIPoint2.pos = math.pi/4
		jointIPoint2.vec = 0
		jointIPoint2.acc = 0
		jointIPoint2.time = 1
		jointGroupPoints1.append(jointIPoint1)
		jointGroupPoints2.append(jointIPoint2)

	traj = trajGenerateByTwoPoints(jointGroupPoints1, jointGroupPoints2)
	print('publish trajectory to the topic')
	#print(traj)
	pub.publish(traj)
	# pos = []
	# vec = []
	# acc = []
	# ti = []
	# for point in traj.points:
	# 	pos.append(point.positions[0])
	# 	vec.append(point.velocities[0])
	# 	acc.append(point.accelerations[0])
	# 	ti.append(point.time_from_start.to_sec())

	# plt.plot(ti, pos, label = 'position')
	# plt.plot(ti, vec, label = 'velocity')
	# plt.plot(ti, acc, label = 'acceleration')
	# plt.legend()
	# plt.show()



