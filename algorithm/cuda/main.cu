#include <algorithm>
#include <fstream>
#include <iostream>
#include <random>
#include <vector>
#include <cuda.h>
#include "robot.hpp"
#include "kernel.cuh"


void d_selection(std::vector<Robot>& robots, double* distances) {
    unsigned int N = robots.size();

    int winner_num = SELECT_PRESSURE * N;
//    printf("winner_num %i\n", winner_num);
    // setup for kernel
    int* d_winners;
    cudaMalloc((void**)&d_winners, winner_num * sizeof(int));
    double* device_distances;
    cudaMalloc((void**)&device_distances, N * sizeof(double));
    cudaMemcpy(device_distances, distances, N * sizeof(double), cudaMemcpyHostToDevice);
    // setup kernel
    curandState* devStates;
    cudaMalloc((void**)&devStates, N * sizeof(devStates));
    // each group will have ten members
    dim3 block(10, 1, 1);
    dim3 grid((int)(N + 10 - 1)/10, 1, 1);
    setup_kernel<<<grid, block>>>(devStates);
    glb_selection<<<grid, block>>>(device_distances, d_winners, devStates);
    cudaDeviceSynchronize();

    int* new_winners = new int[winner_num];
    cudaMemcpy(new_winners, d_winners, winner_num * sizeof(int), cudaMemcpyDeviceToHost);
 
    std::vector<Robot> nextGeneration;
    for (int i = 0; i < winner_num; i++) {
        nextGeneration.push_back(robots[new_winners[i]]);
//        printf("host winners: %i\n", new_winners[i]);
    }
    while (nextGeneration.size() < N) {
        Robot robot(0, 0, 0, 20);
        nextGeneration.push_back(robot);
    }
    robots = nextGeneration;
}




// Genetic algorithm operators, including mutation and crossover!
void evolve(std::vector<Robot>& robots) {
    unsigned int N = robots.size();
    knl_Robot* knl_robots = new knl_Robot[N];

    for (int i = 0; i < N; i++) {
        const Robot& robot = robots[i];
        unsigned int cubenum = robot.existCube.size();
        knl_robots[i].num_cubes = cubenum;
        for (int j = 0; j < cubenum; j++) {
            knl_robots[i].existCubes[j].x = robot.existCube[j][0];
            knl_robots[i].existCubes[j].y = robot.existCube[j][1];
            knl_robots[i].existCubes[j].z = robot.existCube[j][2];
            //printf("x: %f, y: %f, z: %f\n", knl_robot.existCubes[j].x,  knl_robot.existCubes[j].y, knl_robot.existCubes[j].z);
        }
    }
//    printf("Best robot has:  %i cubes\n", knl_robots[0].num_cubes);
    knl_Robot* d_knl_robots;
    cudaMalloc((void**)&d_knl_robots,N * sizeof(knl_Robot)); 
    cudaMemcpy(d_knl_robots, knl_robots, N * sizeof(knl_Robot), cudaMemcpyHostToDevice);

    // set up random 
    curandState *devStates;
    cudaMalloc((void**)&devStates, N * sizeof(devStates));
    dim3 grid((N + 512 - 1) / 512, 1, 1);
    dim3 block(512, 1, 1);
    setup_kernel<<<grid, block>>>(devStates);

    // mutate kernel
    glb_mutate<<<grid, block>>>(d_knl_robots, N, devStates);
    // crossover kernel
    glb_crossover<<<grid, block>>>(d_knl_robots, N, devStates);
    cudaDeviceSynchronize();
    knl_Robot* new_knl_robots = new knl_Robot[N];
    cudaMemcpy(new_knl_robots, d_knl_robots,  N * sizeof(knl_Robot), cudaMemcpyDeviceToHost);
    robots.clear();
    for (int i = 0; i < N; i++) {
        Robot nrobot(new_knl_robots[i].existCubes, new_knl_robots[i].num_cubes);
        robots.push_back(nrobot);
    }
    delete[] new_knl_robots;
    delete[] knl_robots;
}


// Simulating the robots to get their performance/speed
void simulateRobots(const std::vector<Robot>& robots, double* distances) {
    unsigned int N = robots.size();
    // convert CPU data structure to knl optimized data structures
    knl_Robot* knl_robots = new knl_Robot[N];
    for (int i = 0; i < N; ++ i) {
        const Robot& robot = robots[i];
        int num_masses = robot.masses.size();
        int num_springs = robot.springs.size();

        knl_Mass* knl_masses = new knl_Mass[num_masses];
        knl_Spring* knl_springs = new knl_Spring[num_springs];
        for (int j = 0; j < num_masses; ++ j) {
            const Mass& mass = robot.masses[j];
            knl_masses[j].m = mass.m;
            knl_masses[j].a = { mass.a.x, mass.a.y, mass.a.z };
            knl_masses[j].p = { mass.p.x, mass.p.y, mass.p.z };
            knl_masses[j].v = { mass.v.x, mass.v.y, mass.v.z };
            knl_masses[j].force = {mass.force.x, mass.force.y, mass.force.z};
        }
        for (int j = 0; j < num_springs; ++ j) {
            const Spring& spring = robot.springs[j];
            knl_springs[j].a = spring.a;
            knl_springs[j].k = spring.k;
            knl_springs[j].L0 = spring.L0;
            knl_springs[j].m1 = spring.m1;
            knl_springs[j].m2 = spring.m2;
        }
        // prepare for device
        knl_Mass* device_knl_masses;
        cudaMalloc((void**)&device_knl_masses, num_masses * sizeof(knl_Mass));
        cudaMemcpy(device_knl_masses, knl_masses, num_masses * sizeof(knl_Mass), cudaMemcpyHostToDevice);
        knl_Spring* device_knl_spring;
        cudaMalloc((void**)&device_knl_spring, num_springs * sizeof(knl_Spring));
        cudaMemcpy(device_knl_spring, knl_springs, num_springs * sizeof(knl_Spring), cudaMemcpyHostToDevice);

        // save device pointers
        knl_robots[i].num_masses = num_masses;
        knl_robots[i].num_springs = num_springs;
        knl_robots[i].masses = device_knl_masses;
        knl_robots[i].springs = device_knl_spring;

        delete[] knl_masses;
        delete[] knl_springs;
    }

    // put everything else in device
    knl_Robot* device_knl_robots;
    cudaMalloc((void**)&device_knl_robots, N * sizeof(knl_Robot));
    cudaMemcpy(device_knl_robots, knl_robots, N * sizeof(knl_Robot), cudaMemcpyHostToDevice);

    double* device_distances;
    cudaMalloc((void**)&device_distances, N * sizeof(double));

    // run simulation
    dim3 grid((N + 512 - 1) / 512, 1, 1);
    dim3 block(512, 1, 1);
    glb_simulate<<<grid, block>>>(device_knl_robots, N, device_distances);
    cudaDeviceSynchronize();

    // retrieve distances
    cudaMemcpy(distances, device_distances, N * sizeof(double), cudaMemcpyDeviceToHost);

    // free cuda memories
    for (int j = 0; j < N; ++ j) {
        cudaFree(knl_robots[j].masses);
        cudaFree(knl_robots[j].springs);
    }

    cudaFree(device_knl_robots);
    cudaFree(device_distances);

    delete[] knl_robots;
}
std::vector<Robot> geneticAlgorithm(int robotCount, int generationNum, int robotReturn, double cycleTime, bool record) {
    std::vector<Robot> robotGroup = generateRobotGroup(robotCount);
    std::vector<Robot> returnRobot;
    std::ofstream dotchart;
    std::ofstream learningCurve;
    learningCurve.open("/home/jc5667/example2/cube-stuff/algorithm/cuda/learning_curve.txt");
    dotchart.open("/home/jc5667/example2/cube-stuff/algorithm/cuda/dot_chart.txt");
    
    // create a list of robot
    for (int i = 0; i < generationNum; i++) {
        std::cout << "-----------------generation:" << i << "----------------" << std::endl;

        int N = robotGroup.size();
        double* distances = new double[N];
        simulateRobots(robotGroup, distances);
        double greatest_distance = 0;
        for (int j = 0; j < N; ++ j) {
            robotGroup[j].moveDistance = distances[j];
            dotchart << distances[j] << " ";
            printf("distance: %f\n", distances[j]);
            if (robotGroup[j].moveDistance > greatest_distance) {
                greatest_distance = distances[j];
            }
        }
        dotchart << std::endl;
        std::cout << "Greatest distance is: " << greatest_distance << std::endl;
        d_selection(robotGroup, distances);
        delete[] distances;
        // ranking and crossover
        learningCurve << greatest_distance<< std::endl;
//        std::cout << "best in this generation: " << robotGroup[0].moveDistance << std::endl;
        // mutate and crossover
        evolve(robotGroup);
//        std::cout << "finished generation: " << i << std::endl;

    }
    for (int i = 0; i < robotReturn; i++) {
        returnRobot.push_back(robotGroup[i]);
    }
    dotchart.close();
    learningCurve.close();
    return returnRobot;
}
int main(void) {
    std::ofstream parameters;
    parameters.open("/home/jc5667/example2/cube-stuff/algorithm/cuda/parameters.txt");
    std::srand(time(NULL));
    std::vector<Robot> robots = geneticAlgorithm(2000, 3, 10, 5, true);
    for (const auto& robot: robots) {
        for (const auto& cube: robot.existCube) {
            parameters << cube[0] << " " << cube[1] << " " << cube[2] << std::endl;
        }
        parameters << "robot: " << std::endl;
    }
    // TODO


// test for mutation
//    int N = 100;
//    curandState *devStates;
//    cudaMalloc((void**)&devStates, N * sizeof(devStates));
//    dim3 grid((N + 512 - 1) / 512, 1, 1);
//    dim3 block(512, 1, 1);
//    setup_kernel<<<grid, block>>>(devStates);
//
//    std::vector<Robot> robotGroup = generateRobotGroup(N);
//    mutate(robotGroup, devStates);
//    for (int i = 0; i < N; i++) {
//    printf("robotGroup size: %lu\n,  %lu\n ", robotGroup.size(), robotGroup[i].cubes.size());
//    }
    cudaError_t error = cudaGetLastError();
    if(error != cudaSuccess)
    {
    // print the CUDA error message and exit
    printf("CUDA error: %s\n", cudaGetErrorString(error));
    exit(-1);
    }
    cudaDeviceSynchronize();
}
