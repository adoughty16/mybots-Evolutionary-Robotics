from ast import Try
import numpy as np
import pyrosim.pyrosim as pyrosim
import random
import os
import time
import constants as c

class SOLUTION():
	def __init__(self, ID):

		self.myID = ID

		if c.numHiddenNeurons == 3:
			self.sensorsToHidden = np.matrix([[np.random.rand(),np.random.rand(),np.random.rand()],
											  [np.random.rand(),np.random.rand(),np.random.rand()],
											  [np.random.rand(),np.random.rand(),np.random.rand()],
											  [np.random.rand(),np.random.rand(),np.random.rand()]])
			self.hiddenToMotors = np.matrix([[np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand()],
											 [np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand()],
											 [np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand()]])		
		elif c.numHiddenNeurons == 5:
			self.sensorsToHidden = np.matrix([[np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand()],
											  [np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand()],
											  [np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand()],
											  [np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand()]])
			self.hiddenToMotors = np.matrix([[np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand()],
											  [np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand()],
											  [np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand()],
											  [np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand()],
											  [np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand()]])

		self.sensorsToHidden = (self.sensorsToHidden * 2) - 1
		self.hiddenToMotors = (self.hiddenToMotors * 2) - 1


	def Start_Simulation(self, choice):
		#self.Create_World()
		#self.Create_Body()

		if (c.numHiddenNeurons == 3):
			self.Create_Brain()
		if (c.numHiddenNeurons == 5):
			self.Create_Brain2()
		os.system("start /B python simulate.py " + choice + " " + str(self.myID))

	def Wait_For_Simulation_To_End(self):

		fitnessFile = "fitness" + str(self.myID) + ".txt"

		while not os.path.exists(fitnessFile):
			time.sleep(0.01)
		
		maybeError = True

		while (maybeError):
			try:
				fileIn = open(fitnessFile, "r")
			except PermissionError:
				time.sleep(0.01)
			else:
				maybeError = False


		self.fitness = float(fileIn.readline())

		fileIn.close()

		os.system("del " + fitnessFile)

	def Evaluate(self, choice):
		pass

	def Mutate(self):
		if (bool(random.getrandbits(1))):
			self.sensorsToHidden[random.randint(0,c.numSensorNeurons - 1), random.randint(0,c.numHiddenNeurons - 1)] = (random.random() * 2) - 1
		else:
			self.hiddenToMotors[random.randint(0,c.numHiddenNeurons - 1), random.randint(0,c.numMotorNeurons - 1)] = (random.random() * 2) - 1
		
	def Create_World():
		pyrosim.Start_SDF("world.sdf")
		pyrosim.Send_Cube(name="Box", pos=[-10,0,0.1] , size=[15,15,0.2])
		pyrosim.Send_Cube(name="Box", pos=[-10.5,0,0.3] , size=[15,15,0.2])
		pyrosim.Send_Cube(name="Box", pos=[-11,0,0.5] , size=[15,15,0.2])
		pyrosim.Send_Cube(name="Box", pos=[-11.5,0,0.7] , size=[15,15,0.2])
		pyrosim.Send_Cube(name="Box", pos=[-12,0,0.9] , size=[15,15,0.2])
		pyrosim.Send_Cube(name="Box", pos=[-12.5,0,1.1] , size=[15,15,0.2])
		pyrosim.Send_Cube(name="Box", pos=[-13,0,1.3] , size=[15,15,0.2])
		pyrosim.Send_Cube(name="Box", pos=[-13.5,0,1.5] , size=[15,15,0.2])
		pyrosim.Send_Cube(name="Box", pos=[-14,0,1.7] , size=[15,15,0.2])
		pyrosim.Send_Cube(name="Box", pos=[-14.5,0,1.9] , size=[15,15,0.2])
		pyrosim.End()

	def Create_Body():
		pyrosim.Start_URDF("body.urdf")
	
		pyrosim.Send_Cube(name="Torso", pos=[0,0,1] , size=[1,1,1])

		pyrosim.Send_Joint(name="Torso_BackLeg", parent = "Torso", child = "BackLeg",type = "revolute", position = [0,-0.5,1], jointAxis = "1 0 0")
		pyrosim.Send_Cube(name="BackLeg", pos=[0,-0.5,0] , size=[0.2,1,0.2])
		pyrosim.Send_Joint(name="BackLeg_BackShin", parent = "BackLeg", child = "BackShin",type = "revolute", position = [0,-1,0], jointAxis = "1 0 0")
		pyrosim.Send_Cube(name="BackShin", pos=[0,0,-0.5] , size=[0.2,0.2,1])

		pyrosim.Send_Joint(name="Torso_FrontLeg", parent = "Torso", child = "FrontLeg",type = "revolute", position = [0,0.5,1], jointAxis = "1 0 0")
		pyrosim.Send_Cube(name="FrontLeg", pos=[0,0.5,0] , size=[0.2,1,0.2])
		pyrosim.Send_Joint(name="FrontLeg_FrontShin", parent = "FrontLeg", child = "FrontShin",type = "revolute", position = [0,1,0], jointAxis = "1 0 0")
		pyrosim.Send_Cube(name="FrontShin", pos=[0,0,-0.5] , size=[0.2,0.2,1])

		pyrosim.Send_Joint(name="Torso_LeftLeg", parent = "Torso", child = "LeftLeg",type = "revolute", position = [-0.5,0,1], jointAxis = "0 1 0")
		pyrosim.Send_Cube(name="LeftLeg", pos=[-0.5,0,0] , size=[1,0.2,0.2])
		pyrosim.Send_Joint(name="LeftLeg_LeftShin", parent = "LeftLeg", child = "LeftShin",type = "revolute", position = [-1,0,0], jointAxis = "0 1 0")
		pyrosim.Send_Cube(name="LeftShin", pos=[0,0,-0.5] , size=[0.2,0.2,1])

		pyrosim.Send_Joint(name="Torso_RightLeg", parent = "Torso", child = "RightLeg",type = "revolute", position = [0.5,0,1], jointAxis = "0 1 0")
		pyrosim.Send_Cube(name="RightLeg", pos=[0.5,0,0] , size=[1,0.2,0.2])
		pyrosim.Send_Joint(name="RightLeg_RightShin", parent = "RightLeg", child = "RightShin",type = "revolute", position = [1,0,0], jointAxis = "0 1 0")
		pyrosim.Send_Cube(name="RightShin", pos=[0,0,-0.5] , size=[0.2,0.2,1])



		pyrosim.End()

	def Create_Body2(self):
		pyrosim.Start_URDF("body.urdf")
	
		pyrosim.Send_Cube(name="Torso", pos=[0,0,1.25] , size=[1,1,0.5])

		pyrosim.Send_Joint(name="Torso_BackLeg", parent = "Torso", child = "BackLeg",type = "revolute", position = [-0.5,0.375,1.0625], jointAxis = "0 1 0") #Front Left
		pyrosim.Send_Cube(name="BackLeg", pos=[-0.5,0,0] , size=[1,0.2,0.2])
		pyrosim.Send_Joint(name="BackLeg_BackShin", parent = "BackLeg", child = "BackShin",type = "revolute", position = [-1,0,0], jointAxis = "0 1 0")
		pyrosim.Send_Cube(name="BackShin", pos=[0,0,-0.5] , size=[0.2,0.2,1])

		pyrosim.Send_Joint(name="Torso_FrontLeg", parent = "Torso", child = "FrontLeg",type = "revolute", position = [-0.5,-0.375,1.0625], jointAxis = "0 1 0") #Front Right
		pyrosim.Send_Cube(name="FrontLeg", pos=[-0.5,0,0] , size=[1,0.2,0.2])
		pyrosim.Send_Joint(name="FrontLeg_FrontShin", parent = "FrontLeg", child = "FrontShin",type = "revolute", position = [-1,0,0], jointAxis = "0 1 0")
		pyrosim.Send_Cube(name="FrontShin", pos=[0,0,-0.5] , size=[0.2,0.2,1])

		pyrosim.Send_Joint(name="Torso_LeftLeg", parent = "Torso", child = "LeftLeg",type = "revolute", position = [0.5,0.375,1.0625], jointAxis = "0 1 0") #Back Left
		pyrosim.Send_Cube(name="LeftLeg", pos=[0.5,0,0] , size=[1,0.2,0.2])
		pyrosim.Send_Joint(name="LeftLeg_LeftShin", parent = "LeftLeg", child = "LeftShin",type = "revolute", position = [1,0,0], jointAxis = "0 1 0")
		pyrosim.Send_Cube(name="LeftShin", pos=[0,0,-0.5] , size=[0.2,0.2,1])

		pyrosim.Send_Joint(name="Torso_RightLeg", parent = "Torso", child = "RightLeg",type = "revolute", position = [0.5,-0.375,1.0625], jointAxis = "0 1 0") #Back Right
		pyrosim.Send_Cube(name="RightLeg", pos=[0.5,0,0] , size=[1,0.2,0.2])
		pyrosim.Send_Joint(name="RightLeg_RightShin", parent = "RightLeg", child = "RightShin",type = "revolute", position = [1,0,0], jointAxis = "0 1 0")
		pyrosim.Send_Cube(name="RightShin", pos=[0,0,-0.5] , size=[0.2,0.2,1])



		pyrosim.End()

	def Create_Brain(self):
		fileName = "brain" + str(self.myID) + ".nndf"

		pyrosim.Start_NeuralNetwork(fileName)

		pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "BackShin")
		pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "FrontShin")
		pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "LeftShin")
		pyrosim.Send_Sensor_Neuron(name = 3 , linkName = "RightShin")

		pyrosim.Send_Hidden_Neuron( name = 4 )
		pyrosim.Send_Hidden_Neuron( name = 5 )
		pyrosim.Send_Hidden_Neuron( name = 6 )

		pyrosim.Send_Motor_Neuron(name = 7, jointName = "Torso_BackLeg")
		pyrosim.Send_Motor_Neuron(name = 8, jointName = "Torso_FrontLeg")
		pyrosim.Send_Motor_Neuron(name = 9, jointName = "Torso_LeftLeg")
		pyrosim.Send_Motor_Neuron(name = 10, jointName = "Torso_RightLeg")
	
		pyrosim.Send_Motor_Neuron(name = 11, jointName = "BackLeg_BackShin")
		pyrosim.Send_Motor_Neuron(name = 12, jointName = "FrontLeg_FrontShin")
		pyrosim.Send_Motor_Neuron(name = 13, jointName = "LeftLeg_LeftShin")
		pyrosim.Send_Motor_Neuron(name = 14, jointName = "RightLeg_RightShin")

		for currentRow in range (c.numSensorNeurons):
			for currentColumn in range (c.numHiddenNeurons):
				pyrosim.Send_Synapse( sourceNeuronName = currentRow,
							targetNeuronName = currentColumn + c.numSensorNeurons,
							weight = self.sensorsToHidden[currentRow, currentColumn])

		for currentRow in range (c.numHiddenNeurons):
			for currentColumn in range (c.numMotorNeurons):
				pyrosim.Send_Synapse( sourceNeuronName = currentRow + c.numSensorNeurons,
							targetNeuronName = currentColumn + c.numHiddenNeurons + c.numSensorNeurons,
							weight = self.hiddenToMotors[currentRow, currentColumn])

		pyrosim.End()

	def Create_Brain2(self):
		fileName = "brain" + str(self.myID) + ".nndf"

		pyrosim.Start_NeuralNetwork(fileName)

		pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "BackShin")
		pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "FrontShin")
		pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "LeftShin")
		pyrosim.Send_Sensor_Neuron(name = 3 , linkName = "RightShin")

		pyrosim.Send_Hidden_Neuron( name = 4 )
		pyrosim.Send_Hidden_Neuron( name = 5 )
		pyrosim.Send_Hidden_Neuron( name = 6 )
		pyrosim.Send_Hidden_Neuron( name = 7 )
		pyrosim.Send_Hidden_Neuron( name = 8 )

		pyrosim.Send_Motor_Neuron(name = 9, jointName = "Torso_BackLeg")
		pyrosim.Send_Motor_Neuron(name = 10, jointName = "Torso_FrontLeg")
		pyrosim.Send_Motor_Neuron(name = 11, jointName = "Torso_LeftLeg")
		pyrosim.Send_Motor_Neuron(name = 12, jointName = "Torso_RightLeg")

		pyrosim.Send_Motor_Neuron(name = 13, jointName = "BackLeg_BackShin")
		pyrosim.Send_Motor_Neuron(name = 14, jointName = "FrontLeg_FrontShin")
		pyrosim.Send_Motor_Neuron(name = 15, jointName = "LeftLeg_LeftShin")
		pyrosim.Send_Motor_Neuron(name = 16, jointName = "RightLeg_RightShin")

		for currentRow in range (c.numSensorNeurons):
			for currentColumn in range (c.numHiddenNeurons):
				pyrosim.Send_Synapse( sourceNeuronName = currentRow,
							targetNeuronName = currentColumn + c.numSensorNeurons,
							weight = self.sensorsToHidden[currentRow, currentColumn])

		for currentRow in range (c.numHiddenNeurons):
			for currentColumn in range (c.numMotorNeurons):
				pyrosim.Send_Synapse( sourceNeuronName = currentRow + c.numSensorNeurons,
							targetNeuronName = currentColumn + c.numHiddenNeurons + c.numSensorNeurons,
							weight = self.hiddenToMotors[currentRow, currentColumn])

		pyrosim.End()

	def Set_ID(self, ID):
		self.myID = ID