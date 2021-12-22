#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <algorithm>
#include <fstream>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include <random>
#include <vector>
#include <chrono>

const double MASS = 0.1;
double LENGTH = 0.1;
double gravity = 9.81;
double T = 0;
double L0 = 0.1;
double dt = 0.001;
double mu = 0.8;
int DIM = 3;
double omega = 50;
double restoreConstant = 10000;
double kGround = 5000;
int springConstant = 5000;
int motorCube = 5;
double DAMPING = 1;
const double mutatePro = 0.2;
const double crossPro = 0.4;
const double selectPressure = 0.4;
const double overallMass = 10;
extern bool animation;
bool evolution = true;
glm::dvec3 GRAVITY = {0, 0, -9.8};
using namespace std;

unsigned seed = 131;
std::default_random_engine rng(seed);
std::uniform_real_distribution<> dist0(0, 1);
std::uniform_int_distribution<> dist1(8000, 15000);
std::uniform_real_distribution<> dist2(-0.02, 0.02);
std::uniform_int_distribution<> dist3(0, 1);

// glm is a the OpenGL math, glm::dvec3 is a vector with 3 elements
struct Mass {
    double m = MASS;  // mass
    glm::dvec3 p;     // positon x, y, z
    glm::dvec3 v;     // velocity x, y, z
    glm::dvec3 a;     // acceleration
    glm::dvec3 force;
    int useTimes = 1;
};

struct Spring {
    double k = 15000;  // spring constant
    double L0;         // original length
    int m1;            // connected mass 1
    int m2;            // connected mass 2
    double a;
    double b = 0.02;
    double c = 0.01;
    bool operator==(const struct Spring& a) const {
        return (a.m1 == this->m1 && a.m2 == this->m2);
    }
};

struct Cube {
    vector<double> center = {0, 0, 0};
    vector<int> cubeMass;
};


class Robot {
   public:
    // constructor
    Robot(double x, double y, double z, int robotSize);
    Robot(vector<vector<double>> cubes);
    Robot() = default;
    void updateRobot();
    void updateVertices();
    void createCube(double x, double y, double z);
    void someStuffToMakesuretheDrawingWroking();
    bool theSame(Mass m1, Mass m2);
    void breathing();
    void updateSprings();
    glm::dvec2 getPosition();
    double getDistance();
    void setDistance();
    void resetMass();
    void updateCubeFace();
    bool checkExist(double x, double y, double z);
    double moveDistance;
    vector<vector<double>> existCube;  // use to record where exists a cube, help mutate
    vector<Mass> masses;
    vector<Spring> springs;
    vector<int> imMasses;
    vector<Cube> cubes;
    glm::dvec2 startPos;
    glm::dvec2 currentPos;
    int cube_num = 0;
};

Robot mutateRobot(Robot robot);
void selection(vector<Robot>& robotGroup);
vector<Robot> crossoverRobot(Robot parent1, Robot parent2);
int getDiversity(vector<Robot>& robotGroup);
double getGroupDistance(vector<Robot> robot);
vector<Robot> geneticAlgorithm(int robotCount, int generationNum, int robotReturn, double cycleTime, bool record);
void runningSimulate(Robot& robot, double runningTime);
bool approximatelyEqual(double a, double b);
bool springRemove(const Spring& spirng);
bool canDelete(Robot robot, vector<double> cubePosition);
Robot deleteCube(Robot robot, double x, double y, double z);
Robot deleteCubes(Robot robot, vector<vector<double>> positions);

// randomly create a robot, with robotSize number cubes;
Robot::Robot(double x, double y, double z, int robotSize) {
    int randomChoiceCube;
    int randomChoiceFace;
    createCube(x, y, z);
    while (cube_num < robotSize) {
        randomChoiceCube = rand() % cubes.size();
        Cube cube = cubes[randomChoiceCube];
        randomChoiceFace = rand() % 6;
        switch (randomChoiceFace) {
            case 0:
                if (checkExist(cube.center[0] + 0.1, cube.center[1], cube.center[2]))
                    break;
                else {
                    createCube(cube.center[0] + 0.1, cube.center[1], cube.center[2]);
                    break;
                }

            case 1:
                if (checkExist(cube.center[0] - 0.1, cube.center[1], cube.center[2]))
                    break;
                else {
                    createCube(cube.center[0] - 0.1, cube.center[1], cube.center[2]);
                    break;
                }

            case 2:
                if (checkExist(cube.center[0], cube.center[1] + 0.1, cube.center[2]))
                    break;
                else {
                    createCube(cube.center[0], cube.center[1] + 0.1, cube.center[2]);
                    break;
                }

            case 3:
                if (checkExist(cube.center[0], cube.center[1] - 0.1, cube.center[2]))
                    break;
                else {
                    createCube(cube.center[0], cube.center[1] - 0.1, cube.center[2]);
                    break;
                }

            case 4:
                if (checkExist(cube.center[0], cube.center[1], cube.center[2] + 0.1))
                    break;
                else {
                    createCube(cube.center[0], cube.center[1], cube.center[2] + 0.1);
                    break;
                }
            case 5:
                if (checkExist(cube.center[0], cube.center[1], cube.center[2] - 0.1) || cube.center[2] - 0.1 < 0)
                    break;
                else {
                    createCube(cube.center[0], cube.center[1], cube.center[2] - 0.1);
                    break;
                }

            default:
                break;
        }
    }

    startPos = getPosition();
}

// create a robot
Robot::Robot(vector<vector<double>> cubes) {
    for (const vector<double>& cube : cubes) {
        createCube(cube[0], cube[1], cube[2]);
    }
}

// if already exist a cube, this is for generate a cube but not conflict with the previous cube
bool Robot::checkExist(double x, double y, double z) {
    bool exist = false;
    for (const auto& position : existCube) {
        if ((abs(position[0] - x) < 0.001 && abs(position[1] - y) < 0.001 && abs(position[2] - z) < 0.001)) {
            exist = true;
        }
    }
    return exist;
}

// add a cube to the robot
void Robot::createCube(double x, double y, double z) {
    Cube cube;
    cube.center = {x, y, z};
    cube_num += 1;
    vector<double> position = {x, y, z};
    existCube.push_back(position);
    int n = (int)masses.size();
    // first create 8 masses
    vector<Mass> tempMasses(8);
    tempMasses[0] = {MASS, {x + LENGTH / 2, y + LENGTH / 2, z}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    tempMasses[1] = {MASS, {x + LENGTH / 2, y - LENGTH / 2, z}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    tempMasses[2] = {MASS, {x - LENGTH / 2, y - LENGTH / 2, z}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    tempMasses[3] = {MASS, {x - LENGTH / 2, y + LENGTH / 2, z}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    tempMasses[4] = {MASS, {x + LENGTH / 2, y + LENGTH / 2, z + LENGTH}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    tempMasses[5] = {MASS, {x + LENGTH / 2, y - LENGTH / 2, z + LENGTH}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    tempMasses[6] = {MASS, {x - LENGTH / 2, y - LENGTH / 2, z + LENGTH}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    tempMasses[7] = {MASS, {x - LENGTH / 2, y + LENGTH / 2, z + LENGTH}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

    // check the mass created if coinciding with the previous cube
    vector<int> coinciding;
    for (int j = 0; j < tempMasses.size(); j++) {
        for (int i = 0; i < masses.size(); i++) {
            if (theSame(tempMasses[j], masses[i])) {
                masses[i].useTimes += 1;
                tempMasses[j].m = 0;
                coinciding.push_back(i);
                imMasses.push_back(i);
                cube.cubeMass.push_back(i);
            }
        }
    }

    // pushback the not coinciding masses
    int count = 0;
    for (int i = 0; i < tempMasses.size(); i++) {
        if (tempMasses[i].m != 0) {
            masses.push_back(tempMasses[i]);
            imMasses.push_back(n + count);
            cube.cubeMass.push_back(n + count);
            count++;
        }
    }

    // create springs, this is for the nonconinciding masses
    for (int i = n; i < masses.size() - 1; i++) {
        for (int j = i + 1; j < masses.size(); j++) {
            Spring s;
            s.k = springConstant;
            s.m1 = j;
            s.m2 = i;
            springs.push_back(s);
        }
    }

    // create springs, this is for the coinciding masses
    for (int i = 0; i < coinciding.size(); i++) {
        for (int j = n; j < masses.size(); j++) {
            Spring s;
            s.k = springConstant;
            s.m1 = coinciding[i];
            s.m2 = j;
            springs.push_back(s);
        }
    }
    for (Spring& spring : springs) {
        spring.L0 = distance(masses[spring.m1].p, masses[spring.m2].p);
        spring.a = spring.L0;
    }
    cubes.push_back(cube);
}

// the cube in robot breathing, thus the robot can move
void Robot::breathing() {
    for (int i = 0; i < 28; i++) {
        springs[i].L0 = springs[i].a + springs[i].b * sin(omega * T + springs[i].c);
    }
}


// reset the mass of masses, thus every robot has the same total mass
void Robot::resetMass() {
    for (Mass& mass : masses) {
        mass.m = overallMass / double(masses.size());
    }
}

// update the robot per time step according to Newton's law
void Robot::updateRobot() {
    for (int i = 0; i < masses.size(); i++) {
        masses[i].force = {0, 0, 0};
        glm::dvec3 springForce(0.0);
        glm::dvec3 springDirection(0.0);
        double springLength;
        for (const Spring& spring : springs) {
            // find the corresponding spring
            springLength = glm::distance(masses[spring.m1].p, masses[spring.m2].p);
            if (spring.m1 == i) {
                springDirection = glm::normalize(masses[spring.m2].p - masses[spring.m1].p);
                springForce = springDirection * spring.k * (springLength - spring.L0);
                masses[i].force += springForce;
            } else if (spring.m2 == i) {
                springDirection = glm::normalize(masses[spring.m1].p - masses[spring.m2].p);
                springForce = springDirection * spring.k * (springLength - spring.L0);
                masses[i].force += springForce;
            }
        }
        masses[i].force += GRAVITY * MASS;
        if (masses[i].p.z < 0) {
            glm::dvec3 frictionDirection(0.0);
            // static friction
            if (masses[i].v[0] == 0 && masses[i].v[1] == 0) {
                if (masses[i].force[0] == 0 && masses[i].force[1] == 0) {
                    frictionDirection = glm::dvec3(0, 0, 0);
                } else {
                    frictionDirection = glm::normalize(glm::dvec3(-masses[i].force[0], -masses[i].force[1], 0));
                }
            }
            // move friction
            else {
                frictionDirection = glm::normalize(glm::dvec3(-masses[i].v.x, -masses[i].v.y, 0));
            }
            double F_c = -kGround * masses[i].p.z;
            masses[i].force.z += F_c;
            glm::dvec3 frictionForce(0.0);
            frictionForce = frictionDirection * F_c * mu;
            masses[i].force += frictionForce;
        }
    }
    // update accordint to Newton's law, the damping means the speed with decrease with time
    for (int i = 0; i < masses.size(); i++) {
        masses[i].a = masses[i].force / MASS;
        masses[i].v += masses[i].a * dt;
        masses[i].p += masses[i].v * dt;
        masses[i].v *= DAMPING;
    }
}

// simulating the robot, record how fast the robot move
void runningSimulate(Robot& robot, double runningTime) {
    // create a copy
    robot.resetMass();
    Robot tempRobot = robot;
    tempRobot.startPos = tempRobot.getPosition();
    double runTime = 0;
    double dtime = 0.001;

    while (runTime < runningTime) {
        for (int i = 0; i < 28; i++) {
            tempRobot.springs[i].L0 = tempRobot.springs[i].a + 0.02 * sin(omega * runTime + 0.01 * M_PI);
        }
        tempRobot.updateRobot();
        runTime += dtime;
    }
    tempRobot.currentPos = tempRobot.getPosition();
    robot.moveDistance = glm::distance(tempRobot.startPos, tempRobot.currentPos);
    std::cout << "running distance: " << robot.moveDistance << endl;
}

// if two masses are the same, used for avid coinciding
bool Robot::theSame(Mass m1, Mass m2) {
    if (distance(m1.p, m2.p) < 0.00001) {
        return true;
    }
    return false;
}

// get the current position of the robot, the center of mass on xy plane
glm::dvec2 Robot::getPosition() {
    glm::dvec3 pos(0.0);
    glm::dvec2 pos2D(0.0);
    for (const Mass& mass : masses) {
        pos += mass.p;
    }
    pos2D[0] = pos.x;
    pos2D[1] = pos.y;
    glm::dvec2 res = pos2D / (double)masses.size();
    return res;
}

// generate a robot group
vector<Robot> generateRobotGroup(int robotNum) {
    vector<Robot> robotGroup;
    for (int i = 0; i < robotNum; i++) {
        Robot robot(0, 0, 0.0, 20);
        robotGroup.push_back(robot);
    }
    return robotGroup;
}

// crossover two robot
vector<Robot> crossoverRobot(Robot parent1, Robot parent2) {
    vector<Robot> twoChild;
    Robot child1;
    Robot child2;
    double height1 = 0.0;
    double height2 = 0.0;
    // get the height of parents
    for (const vector<double>& exist : parent1.existCube) {
        if (exist[2] > height1) height1 = exist[2];
    }
    for (const vector<double>& exist : parent2.existCube) {
        if (exist[2] > height2) height2 = exist[2];
    }
    // exchangable height
    double height = min(height1, height2);
    int randomChoice = rand() % (int)((height + 0.1) / 0.1) - 1;
    height = 0.1 * double(randomChoice);
    vector<vector<double>> change1;
    vector<vector<double>> change2;
    // all the cubes in the choosen height
    for (const vector<double>& exist : parent1.existCube) {
        if (approximatelyEqual(exist[2], height)) change1.push_back(exist);
    }
    for (const vector<double>& exist : parent2.existCube) {
        if (approximatelyEqual(exist[2], height)) change2.push_back(exist);
    }

    child1 = deleteCubes(parent1, change1);
    for (const vector<double>& change : change2) {
        child1.createCube(change[0], change[1], change[2]);
    }

    child2 = deleteCubes(parent2, change2);
    for (const vector<double>& change : change1) {
        child2.createCube(change[0], change[1], change[2]);
    }
    twoChild.push_back(child1);
    twoChild.push_back(child2);
    return twoChild;
}

// a mutation is add or delete a cube on the robot
Robot mutateRobot(Robot robot) {
    int mutateType = rand() % 2;
    if (robot.cubes.size() > 40) {
        mutateType = 0;
    } else if (robot.cubes.size() < 15) {
        mutateType = 1;
    } else {
        mutateType = mutateType;
    }
    // mutate to delete a cube
    if (mutateType == 0) {
        int mutate1 = 0;
        int times = 0;
        while (mutate1 < 1) {
            int choice = rand() % robot.existCube.size();
            if (canDelete(robot, robot.existCube[choice]) == true) {
                Robot newRobot = deleteCube(robot, robot.existCube[choice][0], robot.existCube[choice][1], robot.existCube[choice][2]);
                mutate1++;
                return newRobot;
            } else {
                choice = rand() % robot.existCube.size();
                times++;
                if (times > 4) {
                    mutateType = 1;
                    mutate1 = 1;
                }
            }
        }
        return robot;
    } 
    // mutate to add a cube
    else {
        int mutate2 = 0;
        while (mutate2 < 1) {
            // choose a cube randomly
            int randomChoiceCube, randomChoiceFace;
            randomChoiceCube = rand() % robot.cubes.size();
            Cube cube = robot.cubes[randomChoiceCube];
            // choose a face randomly
            randomChoiceFace = rand() % 6;
            // create a cube
            switch (randomChoiceFace) {
                case 0:
                    if (robot.checkExist(cube.center[0] + 0.1, cube.center[1], cube.center[2]))
                        break;
                    else {
                        robot.createCube(cube.center[0] + 0.1, cube.center[1], cube.center[2]);
                        mutate2++;
                        break;
                    }

                case 1:
                    if (robot.checkExist(cube.center[0] - 0.1, cube.center[1], cube.center[2]))
                        break;
                    else {
                        robot.createCube(cube.center[0] - 0.1, cube.center[1], cube.center[2]);
                        mutate2++;
                        break;
                    }

                case 2:
                    if (robot.checkExist(cube.center[0], cube.center[1] + 0.1, cube.center[2]))
                        break;
                    else {
                        robot.createCube(cube.center[0], cube.center[1] + 0.1, cube.center[2]);
                        mutate2++;
                        break;
                    }

                case 3:
                    if (robot.checkExist(cube.center[0], cube.center[1] - 0.1, cube.center[2]))
                        break;
                    else {
                        robot.createCube(cube.center[0], cube.center[1] - 0.1, cube.center[2]);
                        mutate2++;
                        break;
                    }

                case 4:
                    if (robot.checkExist(cube.center[0], cube.center[1], cube.center[2] + 0.1))
                        break;
                    else {
                        robot.createCube(cube.center[0], cube.center[1], cube.center[2] + 0.1);
                        mutate2++;
                        break;
                    }
                case 5:
                    if (robot.checkExist(cube.center[0], cube.center[1], cube.center[2] - 0.1) || cube.center[2] - 0.05 < 0)
                        break;
                    else {
                        robot.createCube(cube.center[0], cube.center[1], cube.center[2] - 0.1);
                        mutate2++;
                        break;
                    }
                default:
                    break;
            }
        }
        return robot;
    }
}


// selection is just a buble ranking
void selection(vector<Robot>& robotGroup) {
    int popSize = (int)robotGroup.size();
    for (int i = 0; i < popSize; i++) {
        int swaps = 0;  
        for (int j = 0; j < popSize - i - 1; j++) {
            if (robotGroup[j].moveDistance < robotGroup[j + 1].moveDistance) {
                swap(robotGroup[j], robotGroup[j + 1]);
                swaps = 1; 
            }
        }
        if (!swaps)
            break;  
    }
}

// crossover function, change a layer of the parent robots
void crossover(vector<Robot>& robotGroup) {
    int popSize = (int)robotGroup.size();
    // 0.4 winners will go to next generation
    int winnerNum = popSize * selectPressure;
    vector<Robot> nextGeneration;
    for (int i = 0; i < winnerNum; i++) {
        nextGeneration.push_back(robotGroup[i]);
    }

    // crossover best with others 0.1 in total
    int winnerEffects = 0.05 * popSize;
    for (int i = 0; i < winnerEffects; i++) {
        int randomIndex = rand() % popSize;
        vector<Robot> twoChild;
        twoChild = crossoverRobot(robotGroup[0], robotGroup[randomIndex]);
        nextGeneration.push_back(twoChild[0]);
        nextGeneration.push_back(twoChild[1]);
    }

    // 0.4 intotal
    int crossNum = 0.2 * popSize;
    for (int i = 0; i < crossNum; i++) {
        vector<Robot> twoChild;
        int randomIndex1 = rand() % robotGroup.size();
        int randomIndex2 = rand() % robotGroup.size();
        Robot parent1 = robotGroup[randomIndex1];
        Robot parent2 = robotGroup[randomIndex2];
        twoChild = crossoverRobot(parent1, parent2);
        nextGeneration.push_back(twoChild[0]);
        nextGeneration.push_back(twoChild[1]);
    }

    // generate some new random robot
    while (nextGeneration.size() < popSize) {
        Robot robot(0, 0, 0, 20);
        nextGeneration.push_back(robot);
    }
    robotGroup = nextGeneration;
}


// because delete a cube can be dangerous, we need to know if the cube can be deleted
bool canDelete(Robot robot, vector<double> cubePosition) {
    int nearbyNum = 0;
    if (robot.checkExist(cubePosition[0] + 0.1, cubePosition[1], cubePosition[2])) nearbyNum++;
    if (robot.checkExist(cubePosition[0] - 0.1, cubePosition[1], cubePosition[2])) nearbyNum++;
    if (robot.checkExist(cubePosition[0], cubePosition[1] + 0.1, cubePosition[2])) nearbyNum++;
    if (robot.checkExist(cubePosition[0], cubePosition[1] - 0.1, cubePosition[2])) nearbyNum++;
    if (robot.checkExist(cubePosition[0], cubePosition[1], cubePosition[2] + 0.1)) nearbyNum++;
    if (robot.checkExist(cubePosition[0], cubePosition[1], cubePosition[2] - 0.1)) nearbyNum++;
    if (nearbyNum > 3 || nearbyNum == 1) {
        return true;
    } else {
        return false;
    }
}

// if two numbers are relatively equal
bool approximatelyEqual(double a, double b) {
    return fabs(a - b) <= ((fabs(a) < fabs(b) ? fabs(b) : fabs(a)) * 0.01);
}


// delete a set of cubes, this is used for crossover
Robot deleteCubes(Robot robot, vector<vector<double>> positions) {
    vector<vector<double>> newExist;
    for (const vector<double>& cube : robot.existCube) {
        bool exist = false;
        for (const vector<double>& position : positions) {
            if (approximatelyEqual(cube[0], position[0]) && approximatelyEqual(cube[1], position[1]) && approximatelyEqual(cube[2], position[2])) {
                exist = true;
            }
        }
        if (exist == false) {
            newExist.push_back(cube);
        }
    }
    Robot newRobot(newExist);
    return newRobot;
}

// delete a cube from the robot this is used for mutation
Robot deleteCube(Robot robot, double x, double y, double z) {
    int index = 0;
    for (const vector<double>& cube : robot.existCube) {
        if (approximatelyEqual(cube[0], x) && approximatelyEqual(cube[1], y) && approximatelyEqual(cube[2], z)) {
            break;
        } else {
            index++;
            if (index == robot.cubes.size()) {
                break;
            }
        }
    }
    robot.existCube.erase(robot.existCube.begin() + index);
    Robot newRobot(robot.existCube);
    return newRobot;
}
// genetic algorithm
vector<Robot> geneticAlgorithm(int robotCount, int generationNum, int robotReturn, double cycleTime, bool record) {
    vector<Robot> robotGroup = generateRobotGroup(robotCount);
    vector<Robot> returnRobot;
    ofstream dotchart;
    ofstream learningCurve;
    ofstream parameters;
    learningCurve.open("/home/jc5667/ea/learning_curve.txt");
    dotchart.open("/home/jc5667/ea/dot_chart.txt");

    for (int i = 0; i < generationNum; i++) {
        cout << "-----------------generation:" << i << "----------------" << endl;
        for (int j = 0; j < robotGroup.size(); j++) {
            runningSimulate(robotGroup[j], 2);
            dotchart << robotGroup[j].moveDistance << " ";
        }
        dotchart << endl;
        // selection(decide which robot will enter the next genration)
        selection(robotGroup);
        learningCurve << robotGroup[0].moveDistance << endl;
        cout << "best in this generation: " << robotGroup[0].moveDistance << endl;
        // crossover
        crossover(robotGroup);
        // mutation
        for (int j = 0; j < robotGroup.size(); j++) {
            if (dist0(rng) < mutatePro) {
                robotGroup[j] = mutateRobot(robotGroup[j]);
            }
        }
        cout << "finished generation: " << i << endl;
    }
    for (int i = 0; i < robotReturn; i++) {
        returnRobot.push_back(robotGroup[i]);
    }
    dotchart.close();
    learningCurve.close();
    return returnRobot;
}


int main() { 
     ofstream parameters;
     parameters.open("/home/jc5667/ea/parameters.txt");
     srand(time(NULL));
     // the first parameter the number of robots in each generation
     // the second parameter is the number of generations
     // the third parameter is the number recorded
     // the fourth parameter is the simulating time 
     // the fifth parameter if if the running data will be recorded
     vector<Robot> robots = geneticAlgorithm(400, 1, 10, 2, true);
     for (int i = 0; i < robots.size(); i++) {
     for (const vector<double>& cube: robots[i].existCube) {
         parameters << cube[0] << " " << cube[1] << " " << cube[2] << endl;
     }
     parameters << "Robot: " << i << endl;}
     parameters.close();
}

