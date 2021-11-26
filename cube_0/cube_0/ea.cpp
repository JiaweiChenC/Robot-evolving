//
//  ea.cpp
//  cube_0
//
//  Created by Jiawei Chen on 11/26/21.
//

#include "ea.hpp"
#include "Cube.hpp"

std::random_device dev;
std::mt19937 rng(dev());
std::uniform_real_distribution<> dist0(0, 1);
std::uniform_int_distribution<> dist1(3000, 10000);
std::uniform_real_distribution<> dist2(-0.4, 0.4);

// generate a robot group
vector<Robot> generateRobotGroup(int robotNum) {
    vector<Robot> robotGroup;
    for (int i = 0; i < robotNum; i++) {
        Robot robot(0, 0, 0.1);
        robotGroup.push_back(robot);
    }
    return robotGroup;
}

void mutateRobot(vector<Robot>& robotGroup) {
    for (Robot& robot: robotGroup){
        for (int i = 0; i < DIM * DIM * DIM; i++) {
            if (dist0(rng) < mutatePro){
            robot.gene.k[i] = dist1(rng);
            }
        }
        for (int i = 0; i < DIM * DIM; i++) {
            if (dist0(rng) < mutatePro){
            robot.gene.c[i] = (dist0(rng) * 2 * M_PI);
            robot.gene.b[i] = (dist2(rng));
            }
        }
    }
}

void crossoverRobot(vector<Robot>& robotGroup) {
    // choose two index
    vector<Robot> afterCross;
    int kingEffect = 0.1 * robotGroup.size() ;
    for (int i = 0; i < kingEffect; i++) {
        int randomIndex0 = rand()%robotGroup.size();
        Robot parent1 = robotGroup[0];
        Robot parent2 = robotGroup[randomIndex0];
        if (dist0(rng) < crossPro) {
            std::vector<double> tempk = parent1.gene.k;
            parent1.gene.k = parent2.gene.k;
            parent2.gene.k = tempk;
        }
        // crossover b
        else if (dist0(rng) < crossPro + 0.4) {
            std::vector<double> tempb = parent1.gene.b;
            parent1.gene.b = parent2.gene.b;
            parent2.gene.b = tempb;
        }
        //crossover c
        if (dist0(rng) < crossPro + 0.3) {
            std::vector<double> tempc = parent1.gene.c;
            parent1.gene.c = parent2.gene.c;
            parent2.gene.c = tempc;
        }
        afterCross.push_back(parent1);
        afterCross.push_back(parent2);
    }
    while (afterCross.size() < robotGroup.size())
    {
        int randomIndex1 = rand()%robotGroup.size();
        int randomIndex2 = rand()%robotGroup.size();
        Robot parent1 = robotGroup[randomIndex1];
        Robot parent2 = robotGroup[randomIndex2];
        // cross k
        if (dist0(rng) < crossPro) {
            std::vector<double> tempk = parent1.gene.k;
            parent1.gene.k = parent2.gene.k;
            parent2.gene.k = tempk;
        }
        // crossover b
        else if (dist0(rng) < crossPro + 0.4) {
            std::vector<double> tempb = parent1.gene.b;
            parent1.gene.b = parent2.gene.b;
            parent2.gene.b = tempb;
        }
        //crossover c
        if (dist0(rng) < crossPro + 0.3) {
            std::vector<double> tempc = parent1.gene.c;
            parent1.gene.c = parent2.gene.c;
            parent2.gene.c = tempc;
        }
        afterCross.push_back(parent1);
        afterCross.push_back(parent2);
    }
    robotGroup = afterCross;
}

void selection(vector<Robot>& robotGroup) {
    int popSize = (int)robotGroup.size();
    for(int i = 0; i < popSize; i++) {
       int swaps = 0;         //flag to detect any swap is there or not
       for(int j = 0; j<popSize-i-1; j++) {
          if(robotGroup[j].getDistance() >  robotGroup[j + 1].getDistance()){
             swap(robotGroup[j], robotGroup[j+1]);
             swaps = 1;    //set swap flag
          }
       }
       if(!swaps)
          break;       // No swap in this pass, so array is sorted
    }
    int winnerNum = popSize * selectPressure;
    vector<Robot> nextGeneration;
    for (int i = 0; i < winnerNum; i++) {
        nextGeneration.push_back(robotGroup[i]);
    }
    // generate random robot
    while (nextGeneration.size() < popSize) {
        Robot robot(0, 0, 0.1);
        nextGeneration.push_back(robot);
    }
    robotGroup = nextGeneration;
}

double getDiversity(vector<Robot>& robotGroup) {
    double diversity = 0;
    int n = (int)robotGroup.size();
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            for (int count = 0; count < cube_num; count++) {
                diversity += abs(robotGroup[i].gene.k[count] - robotGroup[j].gene.k[count]);
            }
            for (int num = 0; num < DIM * DIM; num++) {
                diversity += abs(robotGroup[i].gene.c[num] - robotGroup[j].gene.c[num]);
                diversity += abs(robotGroup[i].gene.b[num] - robotGroup[j].gene.b[num]);
            }
        }
    }
    return diversity;
}

vector<Robot> geneticAlgorithm(int robotCount, int generationNum, int robotReturn, double cycleTime) {
    vector<Robot> robotGroup = generateRobotGroup(robotCount);
    vector<Robot> returnRobot;
    double diversity;
    for (int i = 0; i < generationNum; i++) {
        mutateRobot(robotGroup);
        crossoverRobot(robotGroup);
        selection(robotGroup);
        diversity = getDiversity(robotGroup);
    }
    for (int i = 0; i < robotReturn; i++ ) {
        returnRobot.push_back(robotGroup[i]);
    }
    return returnRobot;
}
