//
//  Cube.hpp
//  Boucing_Cube
//
//  Created by Jiawei Chen on 11/7/21.
//

#ifndef Cube_hpp
#define Cube_hpp


#endif /* Cube_hpp */
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
extern double GRAVITY;
extern double SPRINGCONSTANT;
extern double GROUNDCONSTANT;
extern double FRICTION_COEF;
extern double DAMPING;
extern double T;
extern double springPotentialEnergy;
extern double gravityPotentialEnergy;
extern double totalPotentialEnergy;
extern double kineticEnergy;
extern double totalEnergy;
extern double groundEnergy;

using namespace std;

struct Mass
{
    double m;           // mass
    glm::vec3 p;       // positon x, y, z
    double v[3];        // velocity x, y, z
    double a[3];        // acceleration
};

struct Spring
{
    double k;       // spring constant
    double L;      // original length
    int m1;         // connected mass 1
    int m2;         // connected mass 2
};


vector<Mass> generateMass(double mass, double length, double X, double Y, double Z);
vector<Spring> generateSpring(double springConstant);
void cubeUpdate(vector<Mass>& cubeMass, vector<Spring>& cubeSpring, int mode);
double getDistance(double distance_vector[3]);

