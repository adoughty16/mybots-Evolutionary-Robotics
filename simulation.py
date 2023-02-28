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
    
    def Run(self):           
        for i in range(1000):
            p.stepSimulation()
            self.robot.Sense(i)
            self.robot.Act(i)
            
            time.sleep(1/60)

    def __del__(self):
        p.disconnect()
    


