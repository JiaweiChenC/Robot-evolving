#include "Cube.hpp"
double mass = 0.1;
double LENGTH = 0.1;
double gravity = 9.81;
double T = 0;
double L0 = 0.1;
double dt = 0.002;
double restoreConstant = 10000;
double springConstant = 5000;
double DAMPING = 0.9;
double frictionCoefficient = 0;//0.8;
int DIM = 3;
using namespace std;

vector<Mass> masses;
vector<Spring> springs;
vector<int> imMasses;

vector<Mass> generateMass(double mass, double LENGTH, double initialX, double initialY, double initialZ)
{
    vector<Mass> mass_on_cube(8);
    mass_on_cube[0] = {mass,{initialX+LENGTH/2,initialY+LENGTH/2,initialZ},{0,0,0},{0,0,0}};
    mass_on_cube[1] = {mass,{initialX+LENGTH/2,initialY-LENGTH/2,initialZ},{0,0,0},{0,0,0}};
    mass_on_cube[2] = {mass,{initialX-LENGTH/2,initialY-LENGTH/2,initialZ},{0,0,0},{0,0,0}};
    mass_on_cube[3] = {mass,{initialX-LENGTH/2,initialY+LENGTH/2,initialZ},{0,0,0},{0,0,0}};
    mass_on_cube[4] = {mass,{initialX+LENGTH/2,initialY+LENGTH/2,initialZ+LENGTH},{0,0,0},{0,0,0}};
    mass_on_cube[5] = {mass,{initialX+LENGTH/2,initialY-LENGTH/2,initialZ+LENGTH},{0,0,0},{0,0,0}};
    mass_on_cube[6] = {mass,{initialX-LENGTH/2,initialY-LENGTH/2,initialZ+LENGTH},{0,0,0},{0,0,0}};
    mass_on_cube[7] = {mass,{initialX-LENGTH/2,initialY+LENGTH/2,initialZ+LENGTH},{0,0,0},{0,0,0}};
    return mass_on_cube;
}


std::vector<Spring> generateSpring(double springConstant)
{
    double l_cube = 0.1;
    double diagonal1 = sqrt(2) * l_cube;
    double diagonal2 = sqrt(3) * l_cube;
    std::vector<Spring> cubeSprings(28);
    cubeSprings[0] = {springConstant,l_cube,   0,1};
    cubeSprings[1] = {springConstant,diagonal1,0,2};
    cubeSprings[2] = {springConstant,l_cube,   0,3};
    cubeSprings[3] = {springConstant,l_cube,   0,4};
    cubeSprings[4] = {springConstant,diagonal1,0,5};
    cubeSprings[5] = {springConstant,diagonal2,0,6};
    cubeSprings[6] = {springConstant,diagonal1,0,7};

    cubeSprings[7] = {springConstant,l_cube,    1,2};
    cubeSprings[8] = {springConstant,diagonal1, 1,3};
    cubeSprings[9] = {springConstant,diagonal1, 1,4};
    cubeSprings[10] = {springConstant,l_cube,   1,5};
    cubeSprings[11] = {springConstant,diagonal1,1,6};
    cubeSprings[12] = {springConstant,diagonal2,1,7};

    cubeSprings[13] = {springConstant,l_cube,   2,3};
    cubeSprings[14] = {springConstant,diagonal2,2,4};
    cubeSprings[15] = {springConstant,diagonal1,2,5};
    cubeSprings[16] = {springConstant,l_cube,   2,6};
    cubeSprings[17] = {springConstant,diagonal1,2,7};

    cubeSprings[18] = {springConstant,diagonal1,3,4};
    cubeSprings[19] = {springConstant,diagonal2,3,5};
    cubeSprings[20] = {springConstant,diagonal1,3,6};
    cubeSprings[21] = {springConstant,l_cube,   3,7};

    cubeSprings[22] = {springConstant,l_cube,   4,5};
    cubeSprings[23] = {springConstant,diagonal1,4,6};
    cubeSprings[24] = {springConstant,l_cube,   4,7};

    cubeSprings[25] = {springConstant,l_cube,   5,6};
    cubeSprings[26] = {springConstant,diagonal1,5,7};

    cubeSprings[27] = {springConstant,l_cube,6,7};
    return cubeSprings;
}

// calcualte norm for vector
double norm( double x[], std::size_t sz )
{
    double res = 0;
    for (size_t i = 0; i < sz; i ++){
        res += x[i] * x[i];
    }
    return sqrt(res);
}

bool theSame(Mass m1, Mass m2){
    if (distance(m1.p, m2.p) < 0.00001){
        return true;
    }
    return false;
}

void cubeUpdate(std::vector<Mass>& mass_on_cube, std::vector<Spring>& cubeSprings, int option)
{
    // initialize the force vector with value 0
    std::vector<std::vector<double>> cubeForces((int)mass_on_cube.size(),std::vector<double>(3));
    // loop through all springs to calculate spring forces
    for (int i = 0; i < (int)cubeSprings.size(); i++)
    {
        // option 1 to have breathing cube
        Mass mass1 = mass_on_cube[cubeSprings[i].m1];
        Mass mass2 = mass_on_cube[cubeSprings[i].m2];
        double positionDiff[3] = {mass2.p[0] - mass1.p[0],mass2.p[1] - mass1.p[1],mass2.p[2] - mass1.p[2]};
        double L = norm(positionDiff,3);
        double force = cubeSprings[i].k * fabs(cubeSprings[i].L - L);
        double direstion[3] = {positionDiff[0]/L,positionDiff[1]/L,positionDiff[2]/L};
        // contraction case
        if (L > cubeSprings[i].L)
        {
            cubeForces[cubeSprings[i].m1][0] = cubeForces[cubeSprings[i].m1][0] + direstion[0]*force;
            cubeForces[cubeSprings[i].m1][1] = cubeForces[cubeSprings[i].m1][1] + direstion[1]*force;
            cubeForces[cubeSprings[i].m1][2] = cubeForces[cubeSprings[i].m1][2] + direstion[2]*force;
            cubeForces[cubeSprings[i].m2][0] = cubeForces[cubeSprings[i].m2][0] - direstion[0]*force;
            cubeForces[cubeSprings[i].m2][1] = cubeForces[cubeSprings[i].m2][1] - direstion[1]*force;
            cubeForces[cubeSprings[i].m2][2] = cubeForces[cubeSprings[i].m2][2] - direstion[2]*force;
        }
        // expansion case
        else if (L < cubeSprings[i].L)
        {
            cubeForces[cubeSprings[i].m1][0] = cubeForces[cubeSprings[i].m1][0] - direstion[0]*force;
            cubeForces[cubeSprings[i].m1][1] = cubeForces[cubeSprings[i].m1][1] - direstion[1]*force;
            cubeForces[cubeSprings[i].m1][2] = cubeForces[cubeSprings[i].m1][2] - direstion[2]*force;
            cubeForces[cubeSprings[i].m2][0] = cubeForces[cubeSprings[i].m2][0] + direstion[0]*force;
            cubeForces[cubeSprings[i].m2][1] = cubeForces[cubeSprings[i].m2][1] + direstion[1]*force;
            cubeForces[cubeSprings[i].m2][2] = cubeForces[cubeSprings[i].m2][2] + direstion[2]*force;
        }
        // calcualte spring potential energy

    }
    // loop through all masses
    for (int i = 0; i < (int)mass_on_cube.size(); i++)
    {
        // add gravity
        cubeForces[i][2] = cubeForces[i][2] - mass_on_cube[i].m * gravity;
        // cout << "cubeForces Y: " << cubeForces[i][2] << endl;
        // if the mass is below ground, add restroration force and calculate friction
        if (mass_on_cube[i].p[2] <= 0)
        {
            cubeForces[i][2] = cubeForces[i][2] + restoreConstant * fabs(mass_on_cube[i].p[2]);

        }
        // update acceleration
        mass_on_cube[i].a[0] = cubeForces[i][0]/mass_on_cube[i].m;
        mass_on_cube[i].a[1] = cubeForces[i][1]/mass_on_cube[i].m;
        mass_on_cube[i].a[2] = cubeForces[i][2]/mass_on_cube[i].m;
        // update velocity
        mass_on_cube[i].v[0] = mass_on_cube[i].v[0] + mass_on_cube[i].a[0] * dt;
        mass_on_cube[i].v[1] = mass_on_cube[i].v[1] + mass_on_cube[i].a[1] * dt;
        mass_on_cube[i].v[2] = mass_on_cube[i].v[2] + mass_on_cube[i].a[2] * dt;
        // update position
        mass_on_cube[i].p[0] = mass_on_cube[i].p[0] + mass_on_cube[i].v[0] * dt;
        mass_on_cube[i].p[1] = mass_on_cube[i].p[1] + mass_on_cube[i].v[1] * dt;
        mass_on_cube[i].p[2] = mass_on_cube[i].p[2] + mass_on_cube[i].v[2] * dt;
        // calculate gravity potential energy

    }

    // update time
    T = T + dt;
    // re-initilize energy variable
}

void createCube (double x, double y, double z) {
    int n = (int)masses.size();
    // int n2 = (int)springs.size();
    // first create 8 masses
    vector<Mass> tempMasses(8);
    tempMasses[0] = {mass,{x+LENGTH/2,y+LENGTH/2,z},{0,0,0},{0,0,0}};
    tempMasses[1] = {mass,{x+LENGTH/2,y-LENGTH/2,z},{0,0,0},{0,0,0}};
    tempMasses[2] = {mass,{x-LENGTH/2,y-LENGTH/2,z},{0,0,0},{0,0,0}};
    tempMasses[3] = {mass,{x-LENGTH/2,y+LENGTH/2,z},{0,0,0},{0,0,0}};
    tempMasses[4] = {mass,{x+LENGTH/2,y+LENGTH/2,z+LENGTH},{0,0,0},{0,0,0}};
    tempMasses[5] = {mass,{x+LENGTH/2,y-LENGTH/2,z+LENGTH},{0,0,0},{0,0,0}};
    tempMasses[6] = {mass,{x-LENGTH/2,y-LENGTH/2,z+LENGTH},{0,0,0},{0,0,0}};
    tempMasses[7] = {mass,{x-LENGTH/2,y+LENGTH/2,z+LENGTH},{0,0,0},{0,0,0}};
    
    // check the mass created if coinciding with the previous cube if is
    // make m = 0 coinciding pushback the mass overlap, imMasses pushback
    // record all the indices
    vector<int> coinciding;
    for (int j = 0; j < tempMasses.size(); j++) {
        for (int i = 0; i < masses.size(); i++) {
            if (theSame(tempMasses[j], masses[i])) {
                tempMasses[j].m = 0;
                coinciding.push_back(i);
                imMasses.push_back(i);
            }
        }
    }
    
    // pushback the not coinciding masses
    int count = 0;
    for (int i = 0; i < tempMasses.size(); i++) {
        if(tempMasses[i].m != 0) {
            masses.push_back(tempMasses[i]);
            imMasses.push_back(n+count);
            count++;
        }
    }
    
    // create springs, this is for the nonconinciding masses
    for (int i = n; i < masses.size() - 1; i++) {
        for (int j = i+1; j < masses.size(); j++) {
            Spring s;
            s.k = springConstant;
            s.m1 = j;
            s.m2 = i;
            s.L = distance(masses[i].p, masses[j].p);
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
            s.L = distance(masses[s.m1].p, masses[s.m2].p);
            springs.push_back(s);
        }
    }
}

// Robot is a set of cube
void createRobot(){
    for (int i = 0; i < DIM; i++){
        for (int j = 0; j < DIM; j++){
            for (int k = 0; k < DIM; k++){
                createCube((double)(i)/10.0, (double)(j)/10.0, (double)(k)/10.0);
            }
        }
    }
}
