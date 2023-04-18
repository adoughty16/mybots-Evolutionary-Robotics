import pybullet as p
import pyrosim.pyrosim as pyrosim
from sensor import SENSOR
from motor import MOTOR
from pyrosim.neuralNetwork import NEURAL_NETWORK
import os
import constants as c
import math

class ROBOT():

    def __init__(self, solutionID):

        self.solutionID = solutionID

        fileName = "brain" + str(solutionID) + ".nndf"

        self.robotId = p.loadURDF("body.urdf")

        pyrosim.Prepare_To_Simulate(self.robotId)
        self.Prepare_To_Sense()
        self.Prepare_To_Act()
        self.nn = NEURAL_NETWORK(fileName)

        os.system("del " + fileName)


    def Prepare_To_Sense(self):
        self.sensors = {}

        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = SENSOR(linkName)

    def Sense(self, timeStep):
        for i in self.sensors:
            self.sensors[i].Get_Value(timeStep)

    def Prepare_To_Act(self):
        self.motors = {}

        for jointName in pyrosim.jointNamesToIndices:
            self.motors[jointName] = MOTOR(jointName, self.robotId)

    def Act(self, desiredAngle):

        for neuronName in self.nn.Get_Neuron_Names():
            if self.nn.Is_Motor_Neuron(neuronName):
                jointName = self.nn.Get_Motor_Neurons_Joint(neuronName)
                byteStringName = jointName.encode("UTF-8")
                desiredAngle = self.nn.Get_Value_Of(neuronName) * c.motorJointRange
                self.motors[byteStringName].Set_Value(desiredAngle)
        
    def Think(self):
        self.nn.Update()
        #self.nn.Print()

    def Get_Fitness(self):
        basePositionAndOrientation = p.getBasePositionAndOrientation(self.robotId)
        basePosition = basePositionAndOrientation[0]
        xPosition = basePosition[0]
        zPosition = basePosition[2]

        if (c.bodyNumber == 1):
            stateOfLinkZero = p.getLinkState(self.robotId,5)
            linkPosition = stateOfLinkZero[0]
            zPos1 = linkPosition[2]

            stateOfLinkZero = p.getLinkState(self.robotId,7)
            linkPosition = stateOfLinkZero[0]
            zPos2 = linkPosition[2]

        else:
            stateOfLinkZero = p.getLinkState(self.robotId,1)
            linkPosition = stateOfLinkZero[0]
            zPos1 = linkPosition[2]

            stateOfLinkZero = p.getLinkState(self.robotId,7)
            linkPosition = stateOfLinkZero[0]
            zPos2 = linkPosition[2]

        diffPos = zPos1 - zPos2


        tmpFile = "tmp" + str(self.solutionID) + ".txt"
        fitnessFile = "fitness" + str(self.solutionID) + ".txt"

        fileOut = open("tmp"+str(self.solutionID)+".txt", "w")

       # fitness = xPosition + diffPos/2 #this rewards the robot for being at an angle
        #fitness = xPosition + zPosition
        fitness = xPosition
        #fitness = (math.sqrt(pow(xPosition,2) + pow(zPosition,2)))# This totally fails because it just rewards distance from the origin in any direction... Woops


        fileOut.write(str(fitness))
        fileOut.close()
        os.system("rename " + tmpFile + " " + fitnessFile)



