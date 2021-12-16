#include <algorithm>
#include <fstream>
#include <iostream>
#include <random>
#include <vector>

#include "robot.hpp"
#include "kernel.cuh"

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
    std::ofstream Diversity;
    std::ofstream learningCurve;
    std::ofstream parameters;
    learningCurve.open("./learning_curve.txt");
    dotchart.open("./dot_chart.txt");
    Diversity.open("./diversity.txt");

    // create a list of robot
    int diversity;
    for (int i = 0; i < generationNum; i++) {
        std::cout << "-----------------generation:" << i << "----------------" << std::endl;
        // diversity = getDiversity(robotGroup);
        // Diversity << diversity << std::endl;
        // std::cout << "diversity: " << diversity << std::endl;

        int N = robotGroup.size();
        double* distances = new double[N];
        simulateRobots(robotGroup, distances);
        for (int j = 0; j < N; ++ j) {
            robotGroup[j].moveDistance = distances[j];
        }
        delete[] distances;

        dotchart << std::endl;
        // ranking and crossover
        selection(robotGroup);
        learningCurve << robotGroup[0].moveDistance << std::endl;
        std::cout << "best in this generation: " << robotGroup[0].moveDistance << std::endl;
        // crossover
        crossover(robotGroup);
        // mutate
        for (int j = 0; j < robotGroup.size(); j++) {
            if (dist0(rng) < MUTATE_PROB) {
                robotGroup[j] = mutateRobot(robotGroup[j]);
            }
        }
        std::cout << "finished generation: " << i << std::endl;
    }
    for (int i = 0; i < robotReturn; i++) {
        returnRobot.push_back(robotGroup[i]);
    }
    dotchart.close();
    Diversity.close();
    learningCurve.close();
    return returnRobot;
}

int main(void) {
    std::srand(time(NULL));
    std::vector<Robot> robots = geneticAlgorithm(1000, 1, 10, 2, true);
    // TODO
}
