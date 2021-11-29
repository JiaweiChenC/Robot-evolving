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
extern int mass_num;
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
};

struct Spring
{
    double k = 12000;       // spring constant
    double L0;      // original length
    int m1;         // connected mass 1
    int m2;         // connected mass 2
    double a;
    double b = 0.02;
    double c = 0.01;
};

class Cube {
public:
    int index;
    vector<Mass> cubeMass;
    vector<Spring> cubeSpring;
    // constructor
    Cube(double x, double y, double z);
};

struct Gene {
    vector<double> k;
    vector<double> b;
    vector<double> c;
};

class Robot {
public:
    // constructor
    Robot(double x, double y, double z);
    void updateRobot();
    void updateVertices();
    void createCube (double x, double y, double z);
    void someStuffToMakesuretheDrawingWroking();
    bool theSame(Mass m1, Mass m2);
    void breathing();
    void updateSprings();
    glm::dvec3 getPosition();
    double getDistance();
    void setDistance();
    double moveDistance;
    vector<vector<double>> existCube;// use to record where exists a cube, help mutate
    vector<Mass> masses;
    vector<Spring> springs;
    vector<int> imMasses;
    int cube_num = 0;
    Gene gene;
    
private:
    glm::dvec3 startPos;
    glm::dvec3 currentPos;
};

vector<Robot> generateRobotGroup(int robotNum);
vector<Robot> generateRobotGroup2(int robotNum);
void mutateRobot(vector<Robot>& robotGroup);
void selection(vector<Robot>& robotGroup);
vector<Robot> crossoverRobot(Robot parent1, Robot parent2);
double getDiversity(vector<Robot>& robotGroup);
double getGroupDistance(vector<Robot> robot);
vector<Robot> geneticAlgorithm(int robotCount, int generationNum, int robotReturn, double cycleTime, bool record);
void runningSimulate(Robot& robot, double runningTime);
#endif /* Cube_hpp */
