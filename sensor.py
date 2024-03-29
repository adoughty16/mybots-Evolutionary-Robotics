import numpy
import pyrosim.pyrosim as pyrosim

class SENSOR():
    def __init__(self, linkName):
        self.linkName = linkName
        self.values = numpy.zeros(1000)

    def Get_Value(self, timeStep):
        self.values[timeStep] = pyrosim.Get_Touch_Sensor_Value_For_Link(self.linkName)

    def Save_Values(self):
        fileName = "data/" + self.linkName + "SensorVals.npy"
        numpy.save(fileName, self.values)