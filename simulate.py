import pybullet as p
import pyrosim.pyrosim as pyrosim
import time
import pybullet_data
import numpy

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())



p.setGravity(0,0,-9.8)

planeId = p.loadURDF("plane.urdf")

robotId = p.loadURDF("body.urdf")


p.loadSDF("world.sdf")

pyrosim.Prepare_To_Simulate(robotId)

frontLegSensorValues = numpy.zeros(200)
torsoSensorValues = numpy.zeros(200)

for i in range(200):
	p.stepSimulation()
	frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")
	torsoSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("Torso")

	time.sleep(1/60)
p.disconnect()

numpy.save("data/frontLegSensorVals.npy", frontLegSensorValues)
numpy.save("data/torsoSensorVals.npy", torsoSensorValues)