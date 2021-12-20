#include <algorithm>
#include <fstream>
#include <iostream>
#include <random>
#include <vector>
#include <cuda.h>
#include "robot.hpp"
#include "kernel.cuh"

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
    printf("knlrobots %i\n", knl_robots[0].num_cubes);
    knl_Robot* d_knl_robots;
    cudaMalloc((void**)&d_knl_robots,N * sizeof(knl_Robot)); 
    cudaMemcpy(d_knl_robots, knl_robots, N * sizeof(knl_Robot), cudaMemcpyHostToDevice);

    //set up random 
    curandState *devStates;
    cudaMalloc((void**)&devStates, N * sizeof(devStates));
    dim3 grid((N + 512 - 1) / 512, 1, 1);
    dim3 block(512, 1, 1);
    setup_kernel<<<grid, block>>>(devStates);

    //mutate kernel
    glb_mutate<<<grid, block>>>(d_knl_robots, N, devStates);
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
    std::ofstream parameters;
    learningCurve.open("./learning_curve.txt");
    dotchart.open("./dot_chart.txt");

    // create a list of robot
    for (int i = 0; i < generationNum; i++) {
        std::cout << "-----------------generation:" << i << "----------------" << std::endl;

        int N = robotGroup.size();
        double* distances = new double[N];
        simulateRobots(robotGroup, distances);
        for (int j = 0; j < N; ++ j) {
            robotGroup[j].moveDistance = distances[j];
            printf("distance: %f\n", distances[j]);
        }
        delete[] distances;
        dotchart << std::endl;
        // ranking and crossover
        selection(robotGroup);
        learningCurve << robotGroup[0].moveDistance << std::endl;
        std::cout << "best in this generation: " << robotGroup[0].moveDistance << std::endl;
        // crossover
//        crossover(robotGroup);
        // mutate
        evolve(robotGroup);
        std::cout << "finished generation: " << i << std::endl;

    }
    for (int i = 0; i < robotReturn; i++) {
        returnRobot.push_back(robotGroup[i]);
    }
    dotchart.close();
    learningCurve.close();
    return returnRobot;
}
int main(void) {
    std::srand(time(NULL));
    std::vector<Robot> robots = geneticAlgorithm(100, 3, 10, 5, true);
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
