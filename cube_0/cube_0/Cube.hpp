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

#include <GLFW/glfw3.h>

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

struct Mass
{
    double m;           // mass
    glm::dvec3 p;       // positon x, y, z
    glm::dvec3 v;        // velocity x, y, z
    glm::dvec3 a;        // acceleration
};

struct Spring
{
    double k = 10000;       // spring constant
    double L0;      // original length
    int m1;         // connected mass 1
    int m2;         // connected mass 2
    double a = L0;
    double b = 0;
    double c = 0;
};


class Cube {
public:
    int index;
    vector<Mass> cubeMass;
    vector<Spring> cubeSpring;
    // constructor
    Cube(double x, double y, double z);
};

class Robot {
    vector<Cube> robotCube;
};

vector<Mass> generateMass(double mass, double length, double X, double Y, double Z);
vector<Spring> generateSpring(double springConstant);
void cubeUpdate(vector<Mass>& cubeMass, vector<Spring>& cubeSpring, int mode);
double getDistance(double distance_vector[3]);
void createCube();
void createRobot();
void updateRobot();
void updateVertices();
void someStuffToMakesuretheDrawingWroking();
bool theSame(Mass m1, Mass m2);
#endif /* Cube_hpp */
