import numpy as np
import pyrosim.pyrosim as pyrosim
import random
import os

class SOLUTION():
	def __init__(self):
		self.weights = np.matrix([[np.random.rand(), np.random.rand()],
                                  [np.random.rand(), np.random.rand()],
                                  [np.random.rand(), np.random.rand()]])
		self.weights = (self.weights * 2) - 1


	def Evaluate(self):
		self.Create_World()
		self.Create_Body()
		self.Create_Brain()

		os.system("python simulate.py")

		
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
		pyrosim.Start_NeuralNetwork("brain.nndf")
	
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