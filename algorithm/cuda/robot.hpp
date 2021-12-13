#ifndef ROBOT_H
#define ROBOT_H

#include <glm/glm.hpp>
#include <vector>
#include <random>

#include "config.hpp"

static unsigned seed = 131;
static std::default_random_engine rng(seed);
static std::uniform_real_distribution<> dist0(0, 1);
static std::uniform_int_distribution<> dist1(8000, 15000);
static std::uniform_real_distribution<> dist2(-0.02, 0.02);

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
};

struct Cube {
    std::vector<double> center = {0, 0, 0};
    std::vector<int> cubeMass;
};

struct Gene {
    std::vector<double> k;
    std::vector<double> b;
    std::vector<double> c;
};

class Robot {
   public:
    // constructor
    Robot(double x, double y, double z, int robotSize);
    Robot(std::vector<std::vector<double>> cubes);
    Robot() = default;
    void updateVertices();
    void createCube(double x, double y, double z);
    bool theSame(Mass m1, Mass m2);
    bool checkExist(double x, double y, double z);
    std::vector<std::vector<double>> existCube;  // use to record where exists a cube, help mutate
    std::vector<Mass> masses;
    std::vector<Spring> springs;
    std::vector<int> imMasses;
    std::vector<Cube> cubes;
    unsigned int cube_num = 0;
    double moveDistance;
    Gene gene;
};

Robot mutateRobot(Robot robot);
void selection(std::vector<Robot>& robotGroup);
std::vector<Robot> crossoverRobot(Robot parent1, Robot parent2);
int getDiversity(std::vector<Robot>& robotGroup);
double getGroupDistance(std::vector<Robot> robot);
std::vector<Robot> geneticAlgorithm(int robotCount, int generationNum, int robotReturn, double cycleTime, bool record);
void runningSimulate(Robot& robot, double runningTime);
bool approximatelyEqual(double a, double b);
bool springRemove(const Spring& spirng);
bool canDelete(Robot robot, std::vector<double> cubePosition);
Robot deleteCube(Robot robot, double x, double y, double z);
Robot deleteCubes(Robot robot, std::vector<std::vector<double>> positions);
std::vector<Robot> generateRobotGroup(int robotNum);
void crossover(std::vector<Robot>& robotGroup);

#endif
