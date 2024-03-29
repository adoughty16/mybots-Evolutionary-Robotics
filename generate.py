import pyrosim.pyrosim as pyrosim
import random


def Create_World():
	pyrosim.Start_SDF("world.sdf")
	pyrosim.Send_Cube(name="Box", pos=[-3,3,0.5] , size=[1,1,1])
	pyrosim.End()

def Create_Robot():
	pass

def Generate_Body():
	pyrosim.Start_URDF("body.urdf")
	
	pyrosim.Send_Cube(name="BackLeg", pos=[0,0,0.5] , size=[1,1,1])
	pyrosim.Send_Joint(name="BackLeg_Torso", parent = "BackLeg", child = "Torso",type = "revolute", position = [0.5,0,1])
	pyrosim.Send_Cube(name="Torso", pos=[0.5,0,0.5] , size=[1,1,1])
	pyrosim.Send_Joint(name="Torso_FrontLeg", parent = "Torso", child = "FrontLeg",type = "revolute", position = [1,0,0])
	pyrosim.Send_Cube(name="FrontLeg", pos=[0.5,0,-0.5] , size=[1,1,1])

	pyrosim.End()

def Generate_Brain():
	pyrosim.Start_NeuralNetwork("brain.nndf")
	
	pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "BackLeg")
	pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "FrontLeg")
	pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "Torso")

	pyrosim.Send_Motor_Neuron(name = 3, jointName = "BackLeg_Torso")
	pyrosim.Send_Motor_Neuron(name = 4, jointName = "Torso_FrontLeg")

	for i in range (3):
		for j in range (3,5):
			pyrosim.Send_Synapse( sourceNeuronName = i , targetNeuronName = j , weight = random.uniform(-1,1) )

	pyrosim.End()

Create_World()
Create_Robot()
Generate_Body()
Generate_Brain()