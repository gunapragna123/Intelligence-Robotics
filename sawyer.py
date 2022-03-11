import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

import pybullet as p
import numpy as np
import copy
import math
import pybullet_data
import random
class sawyer:

	def __init__(self, timeStep=0.01):
		self.timeStep = timeStep
		self.maxVelocity = 10
		# self.maxForce = 20000.
		self.maxForce = 10000
		self.fingerAForce = 2
		self.fingerBForce = 2.5
		self.fingerTipForce = 2
		self.useInverseKinematics = 1
		self.useSimulation = 1
		self.useNullSpace = 21
		self.useOrientation = 1
		self.palmIndex = 20
		#self.kukaGripperIndex = 7
		#lower limits for null space
		self.ll = [-3.0503, -5.1477, -3.8183, -3.0514, -3.0514, -2.9842, -2.9842, -4.7104, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 
			0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.85, 0.34, 0.17]
		#upper limits for null space
		self.ul = [3.0503, 0.9559, 2.2824, 3.0514, 3.0514, 2.9842, 2.9842, 4.7104, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 
			1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 2.15, 1.5, 1.5]
		#joint ranges for null space
		self.jr = [0, 0, 0, 0, 0, 0, 0, 0, 1.4, 1.4, 1.4, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.3, 1.16, 1.33]
		#restposes for null space
		self.rp = [0]*35
		#joint damping coefficents
		self.jd = [0.0001] * 35
		self.arm = [3, 4, 8, 9, 10, 11, 13, 16]
		self.hand = [21, 22, 23, 26, 27, 28, 30, 31, 32, 35, 36 ,37, 39, 40, 41, 44, 45, 46, 48, 49, 50, 53, 54, 55, 58, 61, 64]
		self.js = [3, 4, 8, 9, 10, 11, 13, 16, 21, 22, 23, 26, 27, 28, 30, 31, 32, 35, 36 ,37, 39, 40, 41, 44, 45, 46, 48, 49, 50, 53, 54, 55, 58, 61, 64]
		self.objectId = -100 
		self.sawyerId = -100
		self.readings = []
		self.reset()	

	def reset(self):
		self.sawyerId = p.loadURDF("./sawyer_robot/sawyer_description/urdf/sawyer.urdf",[0,0,-0.5], [0,0,0,3], useFixedBase = 1)
		#self.objectId = p.loadURDF("./mallet/mallet.urdf",[0.91,0.5,0.1], p.getQuaternionFromEuler([0,4.7,3.14]), useFixedBase = 1)  
		#self.sawyerId = objects[0]
		#for i in range (p.getNumJoints(self.sawyerId)):
		#  print(p.getJointInfo(self.sawyerId,i))
		p.resetBasePositionAndOrientation(self.sawyerId, [-0.10000, 0.000000, 0.0000], [0.000000, 0.000000, 0.000000, 1.000000])

				
		self.armInitial = [-0.725201592068791072, 0.039273803429399234, 1.1960756155189447, 0.432204978167223, -1.6019898175193155, 0.7926017146715626, 0.6437789421536294, 0.8425926422906315]
		#self.handInitial = [0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0, 0.34, 0.34]
		self.handInitial = [0.504170244960853, 0.7071289546916115, 0.16999838208938303, 0.5040461357695973, 0.7078923670031343, 0.16999990381127064, 0.37700306631689845, 1.0257163533604112, 0.16999735406424715, 0.3759031961046794, 1.0247407566023718, 0.1700038673680184, 0.5898816687520184, 0.972921380921876, 0.17000268299189583, 0.589996528767868, 0.9733537003805908, 0.16999999411432384, 0.5088669783084007, 0.9733773010317416, 0.16999941898734464, 0.5084958209673566, 0.9734538186653483, 0.16999992294248686, 1.509343554438042, 0.5049084548298286, 0.504833035006519]

		self.jointPositions = self.armInitial + self.handInitial
		self.numJoints = len(self.js)
		for i in range(self.numJoints):
			p.resetJointState(self.sawyerId, self.js[i], self.jointPositions[i])
			p.setJointMotorControl2(self.sawyerId, self.js[i], 
						p.POSITION_CONTROL, 
						targetPosition=self.jointPositions[i], 
						force= self.maxForce)


		self.trayUid = p.loadURDF("./table/table.urdf", [1.1, 0.000000, -0.3], p.getQuaternionFromEuler([(math.pi/2), 0, (math.pi/2)]), useFixedBase = 1, flags = 8)
		#self.trayUid = p.loadURDF("./table/table.urdf", [1.1, 0, -0.3], p.getQuaternionFromEuler([(math.pi/2), 0, (math.pi/2)]), useFixedBase = 1, flags = 8)
		#self.palmPos = [0.85, -0.05, 0.1]
		self.endEffectorAngle = 0

		self.motorNames = []
		self.motorIndices = []

		for i in range(self.numJoints):
			jointInfo = p.getJointInfo(self.sawyerId, i)
			qIndex = jointInfo[3]
			if qIndex > -1:
			#print("motorname")
			#print(jointInfo[1])
				self.motorNames.append(str(jointInfo[1]))
				self.motorIndices.append(i)




	def getActionDimension(self):
		if (self.useInverseKinematics):
			return len(self.motorIndices)
		return 6  #position x,y,z and roll/pitch/yaw euler angles of end effector


	def getObservationDimension(self):
		return len(self.getObservation())

	def getObservation(self):
		observation = []
		state = p.getLinkState(self.sawyerId, self.palmIndex)
		pos = state[0]
		orn = state[1]
		euler = p.getEulerFromQuaternion(orn)
		observation.extend(list(pos))
		observation.extend(list(euler))
		
		return observation


	def applyAction(self, motorCommands, palmPosition, orientation):


		if (self.useInverseKinematics):

			dx = motorCommands[0]
			dy = motorCommands[1]
			dz = motorCommands[2]
			
			ox = motorCommands[3]
			oy = motorCommands[4]
			oz = motorCommands[5]
	
			thumbl = motorCommands[6]
			thumbm = motorCommands[7]
			indexl = motorCommands[8]
			indexm = motorCommands[9]
			middlel = motorCommands[10]
			middlem = motorCommands[11]
			ringl = motorCommands[12]
			ringm = motorCommands[13]
			pinkyl = motorCommands[14]
			pinkym = motorCommands[15]
			#scalar = motorCommands[13]
			state = p.getLinkState(self.sawyerId, self.palmIndex)
			actualPalmPos = state[0]
			#print("pos[2] (getLinkState(kukaEndEffectorIndex)")
			#print(actualEndEffectorPos[2])
			self.palmPos = palmPosition
			self.palmPos[0] = self.palmPos[0] + dx
			if (self.palmPos[0] > 1):
				self.palmPos[0] = 1
			if (self.palmPos[0] < 0.70):
				self.palmPos[0] = 0.70

			self.palmPos[1] = self.palmPos[1] + dy
			if (self.palmPos[1] < -0.22):
				self.palmPos[1] = -0.22
			if (self.palmPos[1] > 0.17):
				self.palmPos[1] = 0.17

			self.palmPos[2] = self.palmPos[2] + dz
			if (self.palmPos[2] < -0.12):
				self.palmPos[2] = -0.12
			if (self.palmPos[2] > 0.25):
				self.palmPos[2] = 0.25

			self.orientation = orientation
			#self.endEffectorAngle = self.endEffectorAngle + da
			pos = self.palmPos

			self.o1 = self.orientation[0] + ox
			if (self.o1 < -math.pi):
				self.o1 = -math.pi
			if (self.o1 > math.pi):
				self.o1 = math.pi

			self.o2 = self.orientation[1] + oy 
			if (self.o2 < -math.pi):
				self.o2 = -math.pi
			if (self.o2 > math.pi):
				self.o2 = math.pi
			self.o3 = self.orientation[2] + oz
			if (self.o3 < -math.pi):
				self.o3 = -math.pi
			if (self.o3 > math.pi):
				self.o3 = math.pi


			orn = p.getQuaternionFromEuler([self.o1, self.o2, self.o3])
			if (self.useNullSpace == 1):
				if (self.useOrientation == 1):
					jointPoses = p.calculateInverseKinematics(self.sawyerId, self.palmIndex, pos,
                                                    orn, self.ll, self.ul, self.jr, self.rp)
				else:
					jointPoses = p.calculateInverseKinematics(self.sawyerId,
                                                    				self.palmIndex,
                                                    				pos,
                                                    				lowerLimits=self.ll,
                                                    				upperLimits=self.ul,
                                                    				jointRanges=self.jr,
                                                    				restPoses=self.rp)
			else:
				if (self.useOrientation == 1): # run here
					jointP = [0]*65
					jointPoses = p.calculateInverseKinematics(self.sawyerId,
                                                    				self.palmIndex,
                                                    				pos,
                                                    				orn,
                                                    				jointDamping=self.jd)
					j=0
					for i in self.js:
						jointP[i] = jointPoses[j]
						j=j+1
					jointPoses = jointP

				else:
					jointPoses = p.calculateInverseKinematics(self.sawyerId, self.palmIndex, pos)

					#print("jointPoses")
					#print(jointPoses)
					#print("self.palmIndex")
					#print(self.palmIndex)
			if (self.useSimulation): #run here

				for i in range(self.numJoints):
					#print(i)
					p.setJointMotorControl2(bodyUniqueId=self.sawyerId,
                                  				jointIndex=i,
                                  				controlMode=p.POSITION_CONTROL,
                                 				targetPosition=jointPoses[i],
                                  				targetVelocity=0,
                                  				force=self.maxForce,
                                  				maxVelocity=self.maxVelocity,
                                  				positionGain=0.03,
                                  				velocityGain=1)
			else:
				#reset the joint state (ignoring all dynamics, not recommended to use during simulation)
				for i in range(self.numJoints):
					p.resetJointState(self.sawyerId, i, jointPoses[i])
			#wrist
			#p.setJointMotorControl2(bodyIndex=self.sawyerId,
			#			jointIndex=16,
			#			controlMode=p.POSITION_CONTROL,
			#			targetPosition=self.endEffectorAngle,
			#			targetVelocity=0,
			#			force=500,
			#			positionGain=0.03,
			#			velocityGain=1)

			#fingers

			#scaler = [thumb, thumb, index, index, middle, middle, ring, ring, pinky, pinky]
			self.thumb(thumbl, thumbm)
			self.indexF(indexl, indexm)
			self.midF(middlel, middlem)
			self.ringF(ringl, ringm)
			self.pinkyF(pinkyl, pinkym)

		else:
			for action in range(len(motorCommands)):
				motor = self.motorIndices[action]
				p.setJointMotorControl2(self.sawyerId,
                                			motor,
                                			p.POSITION_CONTROL,
                                			targetPosition=motorCommands[action],
                                			force=self.maxForce)
		self.handReading = []
		for i in self.hand:
			self.handReading.append(p.getJointState(self.sawyerId, i)[0])
		

		self.O = []
		self.O.append(self.o1)
		self.O.append(self.o2)
		self.O.append(self.o3)
		self.P = []
		self.P.append(self.palmPos[0])
		self.P.append(self.palmPos[1])
		self.P.append(self.palmPos[2])

	######################################################### Hand Direct Control Functions ##########################################################################

	#control the lower joint and middle joint of pinky finger, range both [0.17 - 1.57]
	def pinkyF(self, lower, middle):
		if(lower < 0.17):
			lower = 0.17
		if(lower > 1.57):
			lower = 1.57
		if(middle < 0.17):
			middle = 0.17
		if(middle > 1.57):
			middle = 1.57	
		p.setJointMotorControlArray(bodyIndex=self.sawyerId,
		                        jointIndices=[21, 26, 22, 27],
		                        controlMode=p.POSITION_CONTROL,
		                        targetPositions=[lower, lower, middle, middle],
		                        targetVelocities=[0, 0, 0, 0],
		                        forces=[5000, 5000, 5000, 5000],
		                        positionGains=[0.03, 0.03, 0.03, 0.03],
		                        velocityGains=[1, 1, 1, 1])	

	#control the lower joint and middle joint of ring finger, range both [0.17 - 1.57]
	def ringF(self, lower, middle):
		if(lower < 0.17):
			lower = 0.17
		if(lower > 1.57):
			lower = 1.57
		if(middle < 0.17):
			middle = 0.17
		if(middle > 1.57):
			middle = 1.57
		p.setJointMotorControlArray(bodyIndex=self.sawyerId,
		                        jointIndices=[30, 35, 31, 36],
		                        controlMode=p.POSITION_CONTROL,
		                        targetPositions=[lower, lower, middle, middle],
		                        targetVelocities=[0, 0, 0, 0],
		                        forces=[5000, 5000, 5000, 5000],
		                        positionGains=[0.03, 0.03, 0.03, 0.03],
		                        velocityGains=[1, 1, 1, 1])

	#control the lower joint and middle joint of mid finger, range both [0.17 - 1.57]
	def midF(self, lower, middle):
		if(lower < 0.17):
			lower = 0.17
		if(lower > 1.57):
			lower = 1.57
		if(middle < 0.17):
			middle = 0.17
		if(middle > 1.57):
			middle = 1.57
		p.setJointMotorControlArray(bodyIndex=self.sawyerId,
		                        jointIndices=[39, 44, 40, 45],
		                        controlMode=p.POSITION_CONTROL,
		                        targetPositions=[lower, lower, middle, middle],
		                        targetVelocities=[0, 0, 0, 0],
		                        forces=[5000, 5000, 5000, 5000],
		                        positionGains=[0.03, 0.03, 0.03, 0.03],
		                        velocityGains=[1, 1, 1, 1])

	#control the lower joint and middle joint of index finger, range both [0.17 - 1.57]
	def indexF(self, lower, middle):
		if(lower < 0.17):
			lower = 0.17
		if(lower > 1.57):
			lower = 1.57
		if(middle < 0.17):
			middle = 0.17
		if(middle > 1.57):
			middle = 1.57
		p.setJointMotorControlArray(bodyIndex=self.sawyerId,
		                        jointIndices=[48, 53, 49, 54],
		                        controlMode=p.POSITION_CONTROL,
		                        targetPositions=[lower, lower, middle, middle],
		                        targetVelocities=[0, 0, 0, 0],
		                        forces=[5000, 5000, 5000, 5000],
		                        positionGains=[0.03, 0.03, 0.03, 0.03],
		                        velocityGains=[1, 1, 1, 1])

	#control the lower joint and middle joint of thumb, range: low [0.17 - 1.57], mid [0.34, 1.5]
	def thumb(self, lower, middle):
		if(lower < 0.85):
			lower = 0.85
		if(lower > 1.57):
			lower = 1.57
		if(middle < 0.34):
			middle = 0.34
		if(middle > 1.5):
			middle = 1.5
		p.setJointMotorControlArray(bodyIndex=self.sawyerId,
		                        jointIndices=[58, 61, 64],
		                        controlMode=p.POSITION_CONTROL,
		                        targetPositions=[lower, middle, middle],
		                        targetVelocities=[0, 0, 0],
		                        forces=[500, 500, 500],
		                        positionGains=[0.03, 0.03, 0.03],
		                        velocityGains=[1, 1, 1])






