from ast import Try
import numpy as np
import pyrosim.pyrosim as pyrosim
import random
import os
import time

class SOLUTION():
	def __init__(self, ID):

		self.myID = ID

		self.weights = np.matrix([[np.random.rand(), np.random.rand()],
                                  [np.random.rand(), np.random.rand()],
                                  [np.random.rand(), np.random.rand()]])
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
		self.weights[random.randint(0,2), random.randint(0,1)] = (random.random() * 2) - 1
		
	def Create_World(self):
		pyrosim.Start_SDF("world.sdf")
		pyrosim.Send_Cube(name="Box", pos=[-3,3,0.5] , size=[1,1,1])
		pyrosim.End()

	def Create_Body(self):
		pyrosim.Start_URDF("body.urdf")
	
		pyrosim.Send_Cube(name="BackLeg", pos=[0,0,0.5] , size=[1,1,1])
		pyrosim.Send_Joint(name="BackLeg_Torso", parent = "BackLeg", child = "Torso",type = "revolute", position = [0.5,0,1])
		pyrosim.Send_Cube(name="Torso", pos=[0.5,0,0.5] , size=[1,1,1])
		pyrosim.Send_Joint(name="Torso_FrontLeg", parent = "Torso", child = "FrontLeg",type = "revolute", position = [1,0,0])
		pyrosim.Send_Cube(name="FrontLeg", pos=[0.5,0,-0.5] , size=[1,1,1])

		pyrosim.End()

	def Create_Brain(self):

		fileName = "brain" + str(self.myID) + ".nndf"

		pyrosim.Start_NeuralNetwork(fileName)
	
		pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "BackLeg")
		pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "FrontLeg")
		pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "Torso")

		pyrosim.Send_Motor_Neuron(name = 3, jointName = "BackLeg_Torso")
		pyrosim.Send_Motor_Neuron(name = 4, jointName = "Torso_FrontLeg")

		for currentRow in range (3):
			for currentColumn in range (2):
				pyrosim.Send_Synapse( sourceNeuronName = currentRow,
							targetNeuronName = currentColumn + 3,
							weight = self.weights[currentRow, currentColumn])

		pyrosim.End()

	def Set_ID(self, ID):
		self.myID = ID