# Evolving soft robot with CUDA C++
In this project, we firstly build a physical simulator, with which we measure the performance of soft robots and evolve them to search for the fastest
soft robot! This project, including the algorithems, is implemented completely from scratch, except for the imported libraries.

# Physical simulator with OpenGL
We built a simple physical simulator with OpenGL. All the configurations of our simulator can be found in the `algorithm/cuda/config.hpp` file. The simulator consists of two parts: the environment and the robot. The environment will provide gravity and friction to the soft robot. Each soft robot is consists of a bunch of 
cubes, and each cube is consists of 8 masses and 28 springs. Making the springs in a cube breathing (the length of the springs changed over time), and the 
cube is the so-called motor.

# Overview:
In this project, we evolve the soft robots using GA. We parallelize simulation, mutation, crossover, and selection processes. In simulation, crossover, and mutation. We mostly use one thread to do the genetic algorithm operations for one robot. In selection, we use tournament selection, which is divided the whole population into small groups
and selects the winners inside each group. The winners are chosen based on their fitness(the move distance). We also make use of shared memory in the selection 
process.

# Prerequisites
For the latest C++/NVCC features, we have to use the latest CUDA toolkit instead of the installation provided by the instructors. Therefore:

1. Follow instructions [here](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#handle-uninstallation) to uninstall existing CUDA, or simply start a brand new VM.
2. Download and install the latest CUDA (e.g. [11.5](https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=18.04&target_type=runfile_local) as we used in our project) 

# Instructions to run
There are two directories in the repo, the cube_0 directory is used for the OpenGL visualization, and the algorithm directory has two sub directories in it, a cuda one that contains the parallel implementation, and a cpu one that contains the serial implementation. 

For both algorithms, make sure to install `glm` (e.g. `sudo apt install libglm-dev` in Ubuntu) before compiling. 

To run the algorithm with parallel implementation:
1. enter algorithm/cuda
2. `$ make evolution`
3. `$ ./evolution`
4. The result will be a learning curve of our GA, a dot chart file record the running distance of all the robots in each generation, and the parameters file which record the parameters of the best robot, the data in parameters file can be defined in the OpenGL project (see below), and then you can see how the robot is running.

To run the algorithm with serial implementation:
1. enter algorithm/cpu
2. `$ make evolution`
3. `$ ./evolution`
4. The result will be a learning curve of our GA, a dot chart file record the running distance of all the robots in each generation, and the parameters file which record the parameters of the best robot, the data in parameters file can be import to the OpenGL function, and then you can see how the robot running.

To run the OpenGL visualization: 
1. The OpenGL program is written with Xcode, thus it's a better choice to run this code on Mac OS
2. Install needed library, e.g., GLFW https://www.glfw.org/, and GLM https://github.com/g-truc/glm, The GLAD already exists in the project, but if it doesn't work, you can try to install a GLAD. GLAD https://glad.dav1d.de/; If you have brew, you can use just use brew to install GLFW and GLM
3. Link the library in Xcode, i.e., change the header search path and library search path, and add the OpenGL framework;
4. Build and run the program in Xcode after opening `cube_0.xcodeproj`.
5. To simulate the best robot we found, firstly, enter cube_0/LearnOpenGL/main.cpp, and change the `ifstream(indata)` path to a .txt file path on your computer, and then paste the parameters into the .txt file. The parameters are just a bunch of coordinates decide where exist a cube. The parameters of our best robot is: 

```
0 0 0
-0.1 0 0
0.1 0 0
-0.1 -0.1 0
0 0.1 0
0.1 0 0.1
0.1 -0.1 0.1
-0.1 0.1 0.1
0 0.1 0.1
-0.1 -0.1 0.1
0 -0.1 0.1
```

Alternatively you can run the parallel algorithm itself, and import the data in parameters to the data.txt file.
