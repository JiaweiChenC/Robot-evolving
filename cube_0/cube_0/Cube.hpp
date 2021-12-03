//
//  Cube.hpp
//  Boucing_Cube
//
//  Created by Jiawei Chen on 11/7/21.
//

#ifndef Cube_hpp
#define Cube_hpp

#define GL_SILENCE_DEPRECATION 
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <random>
#include <GLFW/glfw3.h>
#include <fstream>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

extern const double MASS;
extern double LENGTH;
extern glm::dvec3 GRAVITY;
extern double SPRINGCONSTANT;
extern double GROUNDCONSTANT;
extern double FRICTION_COEF;
extern double DAMPING;
extern double T;
extern double L0;
extern double dt;
extern double mu;
extern int springConstant;
extern int DIM;
extern double omega;
extern double kGround;
using namespace std;
extern int robot_num;

extern const double crossPro;
extern const double mutatePro;
extern const double selectPressure;


struct Mass
{
    double m = MASS;           // mass
    glm::dvec3 p;       // positon x, y, z
    glm::dvec3 v;        // velocity x, y, z
    glm::dvec3 a;        // acceleration
    glm::dvec3 force;
    int useTimes = 1;
//    bool operator==(const struct Mass& a) const
//    {
//        return ( a.p == this->p && a.p == this->p );
//    }
};

struct Spring
{
    double k = 15000;       // spring constant
    double L0;      // original length
    int m1;         // connected mass 1
    int m2;         // connected mass 2
    double a;
    double b = 0.02;
    double c = 0.01;
    bool operator==(const struct Spring& a) const
    {
        return (a.m1 == this->m1 && a.m2 == this->m2);
    }
};


struct Cube {
    vector<double> center = {0, 0, 0};
    vector<int> cubeMass;
};

struct Gene {
    vector<double> k;
    vector<double> b;
    vector<double> c;
};

class Robot {
public:
    // constructor
    Robot(double x, double y, double z, int robotSize);
    Robot(vector<vector<double>> cubes);
    Robot() = default;
    void updateRobot();
    void updateVertices();
    void createCube (double x, double y, double z);
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
    vector<vector<double>> existCube;// use to record where exists a cube, help mutate
    vector<Mass> masses;
    vector<Spring> springs;
    vector<int> imMasses;
    vector<Cube> cubes;
    glm::dvec2 startPos;
    glm::dvec2 currentPos;
    int cube_num = 0;
    Gene gene;

};

Robot mutateRobot(Robot robot);
void selection(vector<Robot>& robotGroup);
//vector<Robot> crossoverRobotMotor(Robot parent1, Robot parent2);
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
#endif /* Cube_hpp */

