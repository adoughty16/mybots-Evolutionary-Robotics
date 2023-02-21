import numpy
import constants as c
import pyrosim.pyrosim as pyrosim
import pybullet as p

class MOTOR():
    def __init__(self, jointName, robotId):
        self.jointName = jointName
        self.robotId = robotId
        self.Prepare_To_Act()

    def Prepare_To_Act(self):
         self.motorValues = numpy.zeros(1000)
         self.amplitude = c.BL_AMPLITUDE
         self.frequency = c.BL_FREQUENCY
         self.offset = c.BL_PHASEOFFSET

         if (self.jointName == b'BackLeg_Torso'):
            self.frequency = c.BL_FREQUENCY*2

    def Set_Value(self, timeStep):

        self.motorValues[timeStep] = self.amplitude * (numpy.sin(self.frequency * timeStep + self.offset))

        pyrosim.Set_Motor_For_Joint(
            bodyIndex = self.robotId,
            jointName = self.jointName,
            controlMode = p.POSITION_CONTROL,
            targetPosition = self.motorValues[timeStep],
            maxForce = 50)

    def Save_Values(self):
        fileName = "data/" + self.jointName + "TargetAngles.npy"
        numpy.save(fileName, self.motorValues)