import numpy
import matplotlib.pyplot as m

#m.plot(numpy.load("data/frontLegSensorVals.npy"), label = "front leg", linewidth = 4)
#m.plot(numpy.load("data/torsoSensorVals.npy"), label = "torso", linewidth = 1)

m.plot(numpy.load("data/BL_targetAngles.npy"), label = "BL target Angles", linewidth = 1)
m.plot(numpy.load("data/targetAngles.npy"), label = "FL target Angles", linewidth = 1)


m.legend()
m.show()