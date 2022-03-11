import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
print("current_dir=" + currentdir)
os.sys.path.insert(0, currentdir)

import math
import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
from sawyer import sawyer
import time
import pybullet as p
import random
import pybullet_data

largeValObservation = 100
RENDER_HEIGHT = 720
RENDER_WIDTH = 960

class sawyerEnv(gym.Env):

	metadata = {'render.modes': ['human', 'rgb_array'], 'video.frames_per_second': 50}

	def __init__(self, urdfRoot=pybullet_data.getDataPath(), 
		actionRepeat=1, 
		isEnableSelfCollision=True, 
		renders=False, 
		isDiscrete=False, 
		maxSteps=1000,
		palmPosition = [0.85, -0.05, 0.1],
		orientation = [0, math.pi*0.5, 0]):

		self.palmPosition = palmPosition
		self.orientation = orientation
		self._isDiscrete = isDiscrete
		# self._timeStep = 1. / 240.
		self._timeStep = 1/200
		self._urdfRoot = urdfRoot
		self._actionRepeat = actionRepeat
		self._isEnableSelfCollision = isEnableSelfCollision
		self._observation = []
		self._envStepCounter = 0
		self._renders = renders
		self._maxSteps = maxSteps
		self.terminated = 0
		self._cam_dist = 1.3
		self._cam_yaw = 180
		self._cam_pitch = -40
		self._sawyerId = -1

		self._p = p
		if self._renders:
			cid = p.connect(p.SHARED_MEMORY)
			if (cid < 0):
				cid = p.connect(p.GUI)
			p.resetDebugVisualizerCamera(1.3, 180, -41, [0.52, -0.2, -0.33])
		else:
			p.connect(p.DIRECT)

		#self.seed()
		self.reset()
		observationDim = len(self.getExtendedObservation())
		observation_high = np.array([largeValObservation] * observationDim)
		if (self._isDiscrete):
			self.action_space = spaces.Discrete(7)
		else:
			action_dim = 3
			self._action_bound = 1
			action_high = np.array([self._action_bound] * action_dim)
			self.action_space = spaces.Box(-action_high, action_high)
		self.observation_space = spaces.Box(-observation_high, observation_high)
		self.viewer = None


	def reset(self):

		self.terminated = 0
		p.resetSimulation()
		p.setPhysicsEngineParameter(numSolverIterations=150)
		p.setTimeStep(self._timeStep)
		
		p.loadURDF(os.path.join(self._urdfRoot, "plane.urdf"), [0, 0, -1])


		p.setGravity(0, 0, -10)

		self._sawyer = sawyer(timeStep=self._timeStep)

		############################################  load tray #######################################################
		# tray
		tray_x = random.uniform(0.8, 1.2)

		print('tray_x: ', tray_x)
		tray_y = random.uniform(-0.5, 0.5)
		print('tray_y: ', tray_y)
		random.seed(30)

		trayId = p.loadURDF("./tray/tray.urdf", [tray_x, tray_y, 0], [0, 0, 0, 3])


		############################################  Change Object ID here #######################################################


		xpos = 0.95 
		ypos = 0 
		ang = 3.14 * 0.5
		orn = p.getQuaternionFromEuler([0, 0, ang])

		object_path = 'random_urdfs/057/057.urdf'
		self.objectId = p.loadURDF(object_path, xpos, ypos, -0.1, orn[0], orn[1], orn[2], orn[3])


		############################################################################################################################
		#p.resetBasePositionAndOrientation(self._sawyerId, [-0.10000, 0.000000, 0.0000], [0.000000, 0.000000, 0.000000, 1.000000])
		self._envStepCounter = 0
		p.stepSimulation()
		self._observation = self.getExtendedObservation()
		return np.array(self._observation)


	def seed(self):
		self.np_random, seed = seeding.np_random(seed)
		return [seed]

	def __del__(self):
		p.disconnect()

	def getExtendedObservation(self):
		# self._observation = [(x, y, z), ([palmContact, thumbContact, indexContact, midContact, ringContact, pinkyContact])]	
		# palm position (x, y, z])
		# contact points in each finger and the palm [palmContact, thumbContact, indexContact, midContact, ringContact, pinkyContact]
		# palmContact = [(linkIndexA, positionOnB, contactDistance, normalForce, lateralFriction1, lateralFrictionDir1, lateralFriction2, lateralFrictionDir2)...()]
		# sum of force 
		#
		palmIndex = 19
		self._observation.append(p.getLinkState(self._sawyer.sawyerId, palmIndex)[0])
		palmContact, thumbContact, indexContact, midContact, ringContact, pinkyContact = self.info()
		self._observation.append([palmContact, thumbContact, indexContact, midContact, ringContact, pinkyContact])
		
		#print("palmContact", palmContact)
		#print("thumbContact", thumbContact)
		#print("indexContact", indexContact)
		#print("midContact", midContact)
		#print("ringContact", ringContact)
		#print("pinkyContact", pinkyContact)

		return self._observation


	def step(self, action):

		dv = 0.005
		dx = action[0] * dv
		dy = action[1] * dv
		da = action[2] * 0.05
		f = 0.3
		realAction = [dx, dy, -0.002, da, f]
		return self.step2(realAction)


	def step2(self, action):

		for i in range(self._actionRepeat):
			self._sawyer.applyAction(action, self.palmPosition, self.orientation)
			p.stepSimulation()
			#if self._termination():
				#break
			self._envStepCounter += 1
		if self._renders:
			time.sleep(self._timeStep)
		self._observation = self.getExtendedObservation()

		#print("self._envStepCounter")
		#print(self._envStepCounter)

		#done = self._termination()

		npaction = np.array([action[3]])  #only penalize rotation until learning works well [action[0],action[1],action[3]])
		actionCost = np.linalg.norm(npaction) * 10.
		#print("actionCost")
		#print(actionCost)
		reward = self._reward() - actionCost
		#print("reward")
		#print(reward)
		#print("len=%r" % len(self._observation))

		#return np.array(self._observation), reward, done, {}

		return np.array(self._observation), reward, {}


	def render(self):
		print("placeholder")

	def _termination(self):
		print("placeholder")

	def _reward(self):
		#rewards is height of target object
		blockPos, blockOrn = p.getBasePositionAndOrientation(self.objectId)
		closestPoints = p.getClosestPoints(self.objectId, self._sawyer.sawyerId, 1000, -1, self._sawyer.palmIndex)
		reward = -1000
		numPt = len(closestPoints)
		if (numPt >0):
			# distance between the hand and the object
			# the closer the better
			reward = -closestPoints[0][8] * 10
		if (blockPos[2] > 0.2):
			reward = reward + 10000
			print("successfully grasped a block!!!")

		return reward


	def info(self):
		palmContact = []
		thumbContact = []
		indexContact = []
		midContact = []
		ringContact = []
		pinkyContact = []
		palmLinks = [19, 20, 25, 29, 34, 38, 43, 47, 52, 56, 57]
		thumbLinks = [58, 59, 60, 61, 62, 63, 64]
		indexLinks = [48, 49, 50, 51, 53, 54, 55]
		middleLinks = [39, 40, 41, 42, 44, 45, 46]
		ringLinks = [30, 31, 32, 33, 35, 36, 37]
		pinkyLinks = [21, 22, 23, 24, 26, 27, 28]

		contact = p.getContactPoints(self._sawyer.sawyerId, self.objectId)
		nums = len(contact)
		#if (nums == 0):
			#return [], [], [], [], [], []

		for i in range(nums):
			temp = []
			if(contact[i][3] in palmLinks):
				temp.append(contact[i][3])  # link ID
				temp.append(contact[i][6])  # contact positionOnB
				temp.append(contact[i][8])  # contactDistance
				temp.append(contact[i][9])  # normalForce 	
				temp.append(contact[i][10]) # lateralFriction1
				temp.append(contact[i][11]) # lateralFrictionDir1
				temp.append(contact[i][12]) # lateralFriction2
				temp.append(contact[i][13]) # lateralFrictionDir2
				palmContact.append(temp)			

			if(contact[i][3] in thumbLinks):
				temp.append(contact[i][3])
				temp.append(contact[i][6])
				temp.append(contact[i][8])
				temp.append(contact[i][9])
				temp.append(contact[i][10])
				temp.append(contact[i][11])
				temp.append(contact[i][12])
				temp.append(contact[i][13])
				thumbContact.append(temp)

			if(contact[i][3] in indexLinks):
				temp.append(contact[i][3])
				temp.append(contact[i][6])
				temp.append(contact[i][8])
				temp.append(contact[i][9])
				temp.append(contact[i][10])
				temp.append(contact[i][11])
				temp.append(contact[i][12])
				temp.append(contact[i][13])
				indexContact.append(temp)

			if(contact[i][3] in middleLinks):
				temp.append(contact[i][3])
				temp.append(contact[i][6])
				temp.append(contact[i][8])
				temp.append(contact[i][9])
				temp.append(contact[i][10])
				temp.append(contact[i][11])
				temp.append(contact[i][12])
				temp.append(contact[i][13])
				midContact.append(temp)

			if(contact[i][3] in ringLinks):
				temp.append(contact[i][3])
				temp.append(contact[i][6])
				temp.append(contact[i][8])
				temp.append(contact[i][9])
				temp.append(contact[i][10])
				temp.append(contact[i][11])
				temp.append(contact[i][12])
				temp.append(contact[i][13])
				ringContact.append(temp)

			if(contact[i][3] in pinkyLinks):
				temp.append(contact[i][3])
				temp.append(contact[i][6])
				temp.append(contact[i][8])
				temp.append(contact[i][9])
				temp.append(contact[i][10])
				temp.append(contact[i][11])
				temp.append(contact[i][12])
				temp.append(contact[i][13])
				pinkyContact.append(temp)

		return palmContact, thumbContact, indexContact, midContact, ringContact, pinkyContact





	def handReading(self):
		return self._sawyer.handReading



	def o(self):
		return self._sawyer.O


	def p(self):
		return self._sawyer.P




