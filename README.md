# Evolving soft robot with CUDA C++
A genetic algorithm to find the fastest soft robot using the CUDA Architecture (GPU). 
This is done by Ruowang Zhang & Jiawei Chen as a final project for the EECSE 4750 Heterogeneous computing course at Columbia University in 2021 fall 
. In this project, we firstly build a physical simulator, with which we measure the performance of soft robots and evolve them to search for the fastest
soft robot!

# Phisical simulator with OpenGL
We built a physical simulator with OpenGL from scratch. All the configurations of our simulator can be found in the algorithm/cuda/config.hpp file. The simulator consists
of two parts: the environment and the robot. The environment will provide gravity and friction to the soft robot. Each soft robot is consists of a bunch of 
cubes, and each cube is consists of 8 masses and 28 springs. Making the springs in a cube breathing (the length of the springs changed over time), and the 
cube is the so-called motor.

# Overview:
In this project, we evolve the soft robots using GA. We parallel simulation, mutation, crossover, and selection processes. In simulation, crossover, and mutation,
we use one thread to do the genetic operations for one robot. In selection, we use tournament selection, which is divided the whole population into small groups
and selects the winners inside each group. The winners are chosen based on their fitness(the move distance). We also make use of shared memory in the selection 
process.

# How to run this code
To run the algorithm with parallel implementation:
1. enter algorithm/cuda
2. $make evolution
3. $./evolution

To run the algorithm with serial implementation:
1. enter algorithm/cpu
2. $make evolution
3. $./evolution

To run the OpenGL file
1. Link needed library, e.g., GLFW, GLAD, GLM;

# Possible Error Scenario
All the codes are created and tested based on the CUDA 2021, makefile may not work with a different CUDA version.

