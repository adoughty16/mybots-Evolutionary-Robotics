import pybullet as p
import pyrosim.pyrosim as pyrosim
import time
import pybullet_data
import numpy
import math
import random

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())



p.setGravity(0,0,-9.8)

planeId = p.loadURDF("plane.urdf")

robotId = p.loadURDF("body.urdf")


p.loadSDF("world.sdf")

pyrosim.Prepare_To_Simulate(robotId)

frontLegSensorValues = numpy.zeros(1000)
torsoSensorValues = numpy.zeros(1000)

for i in range(1000):
	p.stepSimulation()
	frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")
	torsoSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("Torso")

	pyrosim.Set_Motor_For_Joint(
		bodyIndex = robotId,
		jointName = b"BackLeg_Torso",
		controlMode = p.POSITION_CONTROL,
		targetPosition = math.pi/4.0,
		maxForce = 500)

	pyrosim.Set_Motor_For_Joint(
		bodyIndex = robotId,
		jointName = b"Torso_FrontLeg",
		controlMode = p.POSITION_CONTROL,
		targetPosition = math.pi/4.0,
		maxForce = 500)

	time.sleep(1/60)
p.disconnect()

numpy.save("data/frontLegSensorVals.npy", frontLegSensorValues)
numpy.save("data/torsoSensorVals.npy", torsoSensorValues)