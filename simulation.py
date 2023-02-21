from world import WORLD
from robot import ROBOT
import pybullet as p
import pyrosim.pyrosim as pyrosim
import time
import pybullet_data
import numpy
import math
import random
import constants as c

class SIMULATION():
    def __init__(self):
        self.csClient = p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,-9.8)

        self.world = WORLD()
        self.robot = ROBOT()

        pyrosim.Prepare_To_Simulate(self.robot.robotId)
    
    def Run(self):
        frontLegSensorValues = numpy.zeros(1000)
        torsoSensorValues = numpy.zeros(1000)
        BL_targetAngles = numpy.zeros(1000)
        FL_targetAngles = numpy.zeros(1000)

        for i in range(1000):
            BL_targetAngles[i] = c.BL_AMPLITUDE * (numpy.sin(c.BL_FREQUENCY * i + c.BL_PHASEOFFSET))
            FL_targetAngles[i] = c.FL_AMPLITUDE * (numpy.sin(c.FL_FREQUENCY * i + c.FL_PHASEOFFSET))
            
        numpy.save("data/BL_targetAngles.npy", BL_targetAngles)
        numpy.save("data/FL_targetAngles.npy", FL_targetAngles)


        for i in range(1000):
            p.stepSimulation()
            frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")
            torsoSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("Torso")

            pyrosim.Set_Motor_For_Joint(
            	bodyIndex = self.robot.robotId,
            	jointName = b"BackLeg_Torso",
            	controlMode = p.POSITION_CONTROL,
            	targetPosition = BL_targetAngles[i],
            	maxForce = 50)

            pyrosim.Set_Motor_For_Joint(
            	bodyIndex = self.robot.robotId,
            	jointName = b"Torso_FrontLeg",
            	controlMode = p.POSITION_CONTROL,
            	targetPosition = -FL_targetAngles[i],
            	maxForce = 50)
            time.sleep(1/60)

    



