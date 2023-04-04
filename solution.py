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

		self.weights = np.matrix([[np.random.rand()*2-1, np.random.rand()*2-1, np.random.rand()*2-1, np.random.rand()*2-1,np.random.rand()*2-1, np.random.rand()*2-1, np.random.rand()*2-1, np.random.rand()*2-1],
								  [np.random.rand()*2-1, np.random.rand()*2-1, np.random.rand()*2-1, np.random.rand()*2-1,np.random.rand()*2-1, np.random.rand()*2-1, np.random.rand()*2-1, np.random.rand()*2-1],
								  [np.random.rand()*2-1, np.random.rand()*2-1, np.random.rand()*2-1, np.random.rand()*2-1,np.random.rand()*2-1, np.random.rand()*2-1, np.random.rand()*2-1, np.random.rand()*2-1],
								  [np.random.rand()*2-1, np.random.rand()*2-1, np.random.rand()*2-1, np.random.rand()*2-1,np.random.rand()*2-1, np.random.rand()*2-1, np.random.rand()*2-1, np.random.rand()*2-1],
							  	  [np.random.rand()*2-1, np.random.rand()*2-1, np.random.rand()*2-1, np.random.rand()*2-1,np.random.rand()*2-1, np.random.rand()*2-1, np.random.rand()*2-1, np.random.rand()*2-1],
								  [np.random.rand()*2-1, np.random.rand()*2-1, np.random.rand()*2-1, np.random.rand()*2-1,np.random.rand()*2-1, np.random.rand()*2-1, np.random.rand()*2-1, np.random.rand()*2-1],
								  [np.random.rand()*2-1, np.random.rand()*2-1, np.random.rand()*2-1, np.random.rand()*2-1,np.random.rand()*2-1, np.random.rand()*2-1, np.random.rand()*2-1, np.random.rand()*2-1],
								  [np.random.rand()*2-1, np.random.rand()*2-1, np.random.rand()*2-1, np.random.rand()*2-1,np.random.rand()*2-1, np.random.rand()*2-1, np.random.rand()*2-1, np.random.rand()*2-1],
								  [np.random.rand()*2-1, np.random.rand()*2-1, np.random.rand()*2-1, np.random.rand()*2-1,np.random.rand()*2-1, np.random.rand()*2-1, np.random.rand()*2-1, np.random.rand()*2-1],])
		self.weights = (self.weights * 2) - 1


	def Start_Simulation(self, choice):
		self.Create_World()
		self.Create_Body()
		self.Create_Brain()
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
		self.weights[random.randint(0,c.numSensorNeurons - 1), random.randint(0,c.numMotorNeurons - 1)] = (random.random() * 2) - 1
		
	def Create_World(self):
		pyrosim.Start_SDF("world.sdf")
		pyrosim.Send_Cube(name="Box", pos=[-3,3,0.5] , size=[1,1,1])
		pyrosim.End()

	def Create_Body(self):
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

	def Create_Brain(self):

		fileName = "brain" + str(self.myID) + ".nndf"

		pyrosim.Start_NeuralNetwork(fileName)
	
		pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "BackLeg")
		pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "FrontLeg")
		pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "Torso")
		pyrosim.Send_Sensor_Neuron(name = 3 , linkName = "LeftLeg")
		pyrosim.Send_Sensor_Neuron(name = 4 , linkName = "RightLeg")

		pyrosim.Send_Sensor_Neuron(name = 5 , linkName = "BackShin")
		pyrosim.Send_Sensor_Neuron(name = 6 , linkName = "FrontShin")
		pyrosim.Send_Sensor_Neuron(name = 7 , linkName = "LeftShin")
		pyrosim.Send_Sensor_Neuron(name = 8 , linkName = "RightShin")

		pyrosim.Send_Motor_Neuron(name = 9, jointName = "Torso_BackLeg")
		pyrosim.Send_Motor_Neuron(name = 10, jointName = "Torso_FrontLeg")
		pyrosim.Send_Motor_Neuron(name = 11, jointName = "Torso_LeftLeg")
		pyrosim.Send_Motor_Neuron(name = 12, jointName = "Torso_RightLeg")
	
		pyrosim.Send_Motor_Neuron(name = 13, jointName = "BackLeg_BackShin")
		pyrosim.Send_Motor_Neuron(name = 14, jointName = "FrontLeg_FrontShin")
		pyrosim.Send_Motor_Neuron(name = 15, jointName = "LeftLeg_LeftShin")
		pyrosim.Send_Motor_Neuron(name = 16, jointName = "RightLeg_RightShin")

		for currentRow in range (c.numSensorNeurons):
			for currentColumn in range (c.numMotorNeurons):
				pyrosim.Send_Synapse( sourceNeuronName = currentRow,
							targetNeuronName = currentColumn + c.numSensorNeurons,
							weight = self.weights[currentRow, currentColumn])

		pyrosim.End()

	def Set_ID(self, ID):
		self.myID = ID