# mybots

mybots is a project built for my Evolutionary Robotics class I had with Dr. Josh Bongard.

We built a system of neurons and synapses designed to simulate a neural network of a modular design.
We also built virtual robots of varying complexity in a physics engine and interfaced our neural networks with them.
We used evolutionary algorithms and fitness functions to evolve the robots' neural controllers towards performing various 
tasks in the physics engine.

This code is modular. With it you can build a variety of different robots in the physics engine and these can be paired with a similar variety
of neural network architectures and fitness functions that can be tested in any environment that you might feel like building in the engine.

NOTE: This project is built on a library called pyrosim that I did not write (written by Dr. Bongard). I have deleted that from the repository in the interest of not presenting it as
code that I wrote myself. The remaining code is code that I worked on during the course of the class, but the project will not run without pyrosim. This repository
is meant to serve as an example of a system that I understand, have experience with, and know how to manipulate.
