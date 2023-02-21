from simulation import SIMULATION

simulation = SIMULATION()




#import pybullet as p
#import pyrosim.pyrosim as pyrosim
#import time
#import pybullet_data
#import numpy
#import math
#import random
#import constants as c

#random.seed(time.time())



#frontLegSensorValues = numpy.zeros(1000)
#torsoSensorValues = numpy.zeros(1000)
#BL_targetAngles = numpy.zeros(1000)
#FL_targetAngles = numpy.zeros(1000)


#for i in range(1000):
#	BL_targetAngles[i] = c.BL_AMPLITUDE * (numpy.sin(c.BL_FREQUENCY * i + c.BL_PHASEOFFSET))
#	FL_targetAngles[i] = c.FL_AMPLITUDE * (numpy.sin(c.FL_FREQUENCY * i + c.FL_PHASEOFFSET))


#numpy.save("data/BL_targetAngles.npy", BL_targetAngles)
#numpy.save("data/FL_targetAngles.npy", FL_targetAngles)



#for i in range(1000):
#	p.stepSimulation()
#	frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")
#	torsoSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("Torso")

#	pyrosim.Set_Motor_For_Joint(
#		bodyIndex = robotId,
#		jointName = b"BackLeg_Torso",
#		controlMode = p.POSITION_CONTROL,
#		targetPosition = BL_targetAngles[i],
#		maxForce = 50)

#	pyrosim.Set_Motor_For_Joint(
#		bodyIndex = robotId,
#		jointName = b"Torso_FrontLeg",
#		controlMode = p.POSITION_CONTROL,
#		targetPosition = -FL_targetAngles[i],
#		maxForce = 50)

#	time.sleep(1/60)
#p.disconnect()

#numpy.save("data/frontLegSensorVals.npy", frontLegSensorValues)
#numpy.save("data/torsoSensorVals.npy", torsoSensorValues)