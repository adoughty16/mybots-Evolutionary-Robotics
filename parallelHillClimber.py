import dataclasses
from solution import SOLUTION
import constants as c
import copy
import os

class PARALLEL_HILL_CLIMBER():
    def __init__(self):

        os.system("del brain*.nndf")
        os.system("del fitness*.txt")

        self.nextAvailableID = 0

        self.parents = {}
        for i in range(c.POPULATION_SIZE):
            self.parents[i] = SOLUTION(self.nextAvailableID)
            self.nextAvailableID += 1

    def Evolve(self):

        self.Evaluate(self.parents)

        for currentGeneration in range(c.NUMBER_OF_GENERATIONS):
            self.Evolve_For_One_Generation()

    def Evolve_For_One_Generation(self):
        self.Spawn()
        self.Mutate()
        self.Evaluate(self.children)
        self.Print()
        self.Select()

    def Evaluate(self, solutions):
        for i in solutions:
            solutions[i].Start_Simulation("DIRECT")

        for i in solutions:
            solutions[i].Wait_For_Simulation_To_End()


    def Spawn(self):
        self.children = {}
        for i in self.parents:
            self.children[i] = copy.deepcopy(self.parents[i])
            self.children[i].Set_ID(self.nextAvailableID)
            self.nextAvailableID += 1

    def Mutate(self):
        for i in self.children:
            self.children[i].Mutate()

    def Select(self):
        for i in self.parents:
            if (self.children[i].fitness < self.parents[i].fitness):
                self.parents[i] = self.children[i]

    def Print(self):
        for i in self.parents:
            print("")
            print("\n\n" + str(self.parents[i].fitness) + " --- " + str(self.children[i].fitness) + "\n")
            print("")

    def Show_Best(self):
        bestFit = 999
        bestParent = -1
        for i in self.parents:
            if (self.parents[i].fitness < bestFit):
                bestFit = self.parents[i].fitness
                bestParent = i
        self.parents[bestParent].Start_Simulation("GUI")