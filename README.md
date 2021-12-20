# Evolving soft robot with CUDA C++
A genetic algorithm to find fastest soft robot using the CUDA Architecture (GPU). 
This is done by Ruowang Zhang & Jiawei chen as a final project for the EECSE 4750 Heterougeneous computing course at Columbia University in 2021 fall 
. In this project, we fistly build a phisical simulator, with which we measure the performance of soft robots and evolve them to search for the fatest
soft robot!

# Phisical simulator with OpenGL
We build a phisical simulator with OpenGL from scratch, all the configuration of our simulator can be found in the algorithm/cuda/config.hpp file. The simulator consists
of two parts: environment and the robot. The environment will provide gravity and friction to the soft robot. Each soft robot is consists of a bunch of 
cubes, and each cube is consists of 8 masses and 28 springs. Making the springs in a cube breathing (the length of the springs changed over time), and the 
cube is so called motor.

# Overview:
In this project, we evolve the soft robots using GA. We parallel simulation, mutation, crossover, and selection process, in simulation, crossover, and mutation
we use one thread to do the genetic operations for one robot. In selection, we use tournament selection, which is divided the whole population into small groups
and select the winners inside each group, the winners are selected based on their fitness(the move distance), we also make use of shared memory in the selection 
process.

# How to run this code
To run the algorithm with parallel implemention:
1. enter algorithm/cuda
2. $make evolution
3. $./evolution

To run the algorithm with serial implemention:
1. enter algorithm/cpu
2. $make evolution$
3. $./evolution$

To run the OpenGL simulator
1. Install glm library
2. Link OpenGL library: GLFW and GLAD

