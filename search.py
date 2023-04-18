import os
from parallelHillClimber import PARALLEL_HILL_CLIMBER
from solution import SOLUTION as s
import pybullet as p

#for i in range(5):
#    os.system("python generate.py")
#    os.system("python simulate.py")

s.Create_World()
s.Create_Body()
phc = PARALLEL_HILL_CLIMBER()
phc.Evolve()
input("Hit enter to show best")
phc.Show_Best()

