import pybullet as p
import pyrosim.pyrosim as pyrosim
import time
import pybullet_data
import numpy
import math
import random

BL_AMPLITUDE = math.pi/4
BL_FREQUENCY = 1/10
BL_PHASEOFFSET = 0

FL_AMPLITUDE = math.pi/4
FL_FREQUENCY = 1/30
FL_PHASEOFFSET = 20

random.seed(time.time())
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())



p.setGravity(0,0,-9.8)

planeId = p.loadURDF("plane.urdf")

robotId = p.loadURDF("body.urdf")


p.loadSDF("world.sdf")

pyrosim.Prepare_To_Simulate(robotId)

frontLegSensorValues = numpy.zeros(1000)
torsoSensorValues = numpy.zeros(1000)
BL_targetAngles = numpy.zeros(1000)
FL_targetAngles = numpy.zeros(1000)


for i in range(1000):
	BL_targetAngles[i] = BL_AMPLITUDE * (numpy.sin(BL_FREQUENCY * i + BL_PHASEOFFSET))
	FL_targetAngles[i] = FL_AMPLITUDE * (numpy.sin(FL_FREQUENCY * i + FL_PHASEOFFSET))


numpy.save("data/BL_targetAngles.npy", BL_targetAngles)
numpy.save("data/FL_targetAngles.npy", FL_targetAngles)



for i in range(1000):
	p.stepSimulation()
	frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")
	torsoSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("Torso")

	pyrosim.Set_Motor_For_Joint(
		bodyIndex = robotId,
		jointName = b"BackLeg_Torso",
		controlMode = p.POSITION_CONTROL,
		targetPosition = BL_targetAngles[i],
		maxForce = 50)

	pyrosim.Set_Motor_For_Joint(
		bodyIndex = robotId,
		jointName = b"Torso_FrontLeg",
		controlMode = p.POSITION_CONTROL,
		targetPosition = -FL_targetAngles[i],
		maxForce = 50)

	time.sleep(1/60)
p.disconnect()

numpy.save("data/frontLegSensorVals.npy", frontLegSensorValues)
numpy.save("data/torsoSensorVals.npy", torsoSensorValues)