import pybullet as p
import time

class WORLD():
    def __init__(self):
        self.planeId = p.loadURDF("plane.urdf")
        p.loadSDF("world.sdf")

