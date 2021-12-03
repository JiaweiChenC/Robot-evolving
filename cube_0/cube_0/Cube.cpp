#include "Cube.hpp"
#include <time.h>
#include <assert.h>
#include <algorithm>
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
const double mutatePro = 0.02;
const double crossPro = 0.4;
const double selectPressure = 0.3;
extern bool animation;
bool evolution = true;
glm::dvec3 GRAVITY = {0, 0, -9.8};
vector<GLdouble> cubeVertices; // cube_count * 8 points * 3 (x, y, z)
vector<GLdouble> cubeColor; // cube_count * 8 points * 3 bit
vector<GLdouble> pointColor;
vector<GLuint> cubeIndices;   // cube_count * 6 faces * 2 triangles * 3 indices
vector<GLdouble> myEdge_color;      // cube_count * 8 points * 3 bit
vector<GLuint> myEdge_indices; // cube_count * 12 * 2
vector<GLuint> myShadeindices;

extern vector<vector<GLuint>> faceIndices;
extern vector<vector<GLuint>> edgeIndices;

std::random_device dev;
std::mt19937 rng(dev());
std::uniform_real_distribution<> dist0(0, 1);
std::uniform_int_distribution<> dist1(3000, 15000);
std::uniform_real_distribution<> dist2(-0.01, 0.01);
using namespace std;

Robot::Robot(double x, double y, double z, int robotSize){
    int randomChoiceCube;
    int randomChoiceFace;
    //cout << x << " " << y << " " << z << " " << endl;
    createCube(x, y, z);
//    cout << "the first cube" << endl;
    while(cube_num < robotSize){
        // choose a cube randomly
        randomChoiceCube = rand() % cubes.size();
//        cout << "random choose cube: " << randomChoiceCube << endl;
        Cube cube = cubes[randomChoiceCube];
        // choose a face randomly
        randomChoiceFace = rand() % 6;
//        cout << "random choose face: " << randomChoiceFace << endl;
        // generate a cube in front
        switch (randomChoiceFace) {
            case 0:
                // create a front cube
                if (checkExist(cube.center[0] + 0.1, cube.center[1], cube.center[2])) break;
                else {
                    createCube(cube.center[0] + 0.1, cube.center[1], cube.center[2]);
                    break;
                }

            case 1:
                if (checkExist(cube.center[0] - 0.1, cube.center[1], cube.center[2])) break;
                else {
                    createCube(cube.center[0] - 0.1, cube.center[1], cube.center[2]);
                    break;
                }
                    
            case 2:
                if (checkExist(cube.center[0], cube.center[1] + 0.1, cube.center[2])) break;
                else {
                    createCube(cube.center[0], cube.center[1] + 0.1, cube.center[2]);
                    break;
                }
    
            case 3:
                if (checkExist(cube.center[0], cube.center[1] - 0.1, cube.center[2])) break;
                else {
                    createCube(cube.center[0], cube.center[1] - 0.1, cube.center[2]);
                    break;
                }
                
            case 4:
                if (checkExist(cube.center[0], cube.center[1], cube.center[2] + 0.1)) break;
                else {
                    createCube(cube.center[0], cube.center[1], cube.center[2] + 0.1);
                    break;
                }
            case 5:
                if (checkExist(cube.center[0], cube.center[1], cube.center[2] - 0.1) || cube.center[2] - 0.1<0) break;
                else {
                    createCube(cube.center[0], cube.center[1], cube.center[2] - 0.1);
                    break;
                }

            default:
                break;
        }
    }

    if (evolution) {
    // get the correspond parameters
    // every cube will have a k
        for (int count = 0; count < DIM * DIM; count++) {
            gene.k.push_back(dist1(rng));
        }
        // have motorCube to generate power for cube
        for (int count = 0; count < DIM * DIM; count++) {
            gene.c.push_back(dist0(rng) * 2 * M_PI);
            gene.b.push_back(dist2(rng));
        }

    }
    startPos = getPosition();
}

Robot::Robot(vector<vector<double>> cubes) {
    for (const vector<double>& cube: cubes) {
        createCube(cube[0], cube[1], cube[2]);
    }
}


bool Robot::checkExist(double x, double y, double z) {
    bool exist = false;
    for (const auto& position: existCube){
        if ((abs(position[0] - x) < 0.001 && abs(position[1]- y) < 0.001 && abs(position[2] - z) < 0.001)) {
            exist = true;
        }
    }
    return exist;
}

void Robot::createCube (double x, double y, double z) {
    Cube cube;
    //cout << "cube num" << cubes.size() << endl;
    // this center is a center of xy face
    cube.center = {x, y, z};
    //cout << "x: " << x << " y: " << y << " z: " << z << endl;
    cube_num += 1;
    vector<double> position = {x ,y ,z};
    existCube.push_back(position);
    int n = (int)masses.size();
    // int n2 = (int)springs.size();
    // first create 8 masses
    vector<Mass> tempMasses(8);
    tempMasses[0] = {MASS,{x+LENGTH/2,y+LENGTH/2,z},{0,0,0},{0,0,0},{0,0,0}};
    tempMasses[1] = {MASS,{x+LENGTH/2,y-LENGTH/2,z},{0,0,0},{0,0,0},{0,0,0}};
    tempMasses[2] = {MASS,{x-LENGTH/2,y-LENGTH/2,z},{0,0,0},{0,0,0},{0,0,0}};
    tempMasses[3] = {MASS,{x-LENGTH/2,y+LENGTH/2,z},{0,0,0},{0,0,0},{0,0,0}};
    tempMasses[4] = {MASS,{x+LENGTH/2,y+LENGTH/2,z+LENGTH},{0,0,0},{0,0,0},{0,0,0}};
    tempMasses[5] = {MASS,{x+LENGTH/2,y-LENGTH/2,z+LENGTH},{0,0,0},{0,0,0},{0,0,0}};
    tempMasses[6] = {MASS,{x-LENGTH/2,y-LENGTH/2,z+LENGTH},{0,0,0},{0,0,0},{0,0,0}};
    tempMasses[7] = {MASS,{x-LENGTH/2,y+LENGTH/2,z+LENGTH},{0,0,0},{0,0,0},{0,0,0}};
    
    // check the mass created if coinciding with the previous cube if is
    // make m = 0 coinciding pushback the mass overlap, imMasses pushback
    // record all the indices
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
        if(tempMasses[i].m != 0) {
            masses.push_back(tempMasses[i]);
            imMasses.push_back(n+count);
            cube.cubeMass.push_back(n + count);
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
    for (Spring& spring: springs){
        spring.L0 = distance(masses[spring.m1].p, masses[spring.m2].p);
        spring.a= distance(masses[spring.m1].p, masses[spring.m2].p);
    }
    cubes.push_back(cube);
    //cout << "coinciding: " << coinciding.size() << endl;
    //cout << "imMasses: ";
//    for (const auto& i: imMasses) {
//        cout  << i << " " ;
//    }
    //cout << "imMasses size: " << imMasses.size() << endl;
}

void Robot::someStuffToMakesuretheDrawingWroking() {
    for (int i = 0; i < cubes.size(); i++) {
        vector<int> tempCube;
        vector<int> upper;
        vector<int> down;
        vector<int> front;
        vector<int> back;
        int right;
        int left;
        for (int j = 0; j < 8; j++) {
            tempCube.push_back(imMasses[8 * i + j]);
        }
        for (int n = 0; n < 8; n++) {
            if (abs(masses[tempCube[n]].p.z - masses[tempCube[0]].p.z) < 0.01){
                upper.push_back(tempCube[n]);
            }
            else {
                down.push_back(tempCube[n]);
            }
        }
        assert(upper.size() == 4);
        assert(down.size() == 4);
//        cout << upper[0]<<  endl;
        //  flip the position if wrong
        if (masses[upper[0]].p.z < masses[down[0]].p.z) {
            vector<int> temp = upper;
            upper = down;
            down = temp;
        }
        
        for (int n = 0; n < 4; n++) {
            if (abs(masses[upper[n]].p.y - masses[upper[0]].p.y) < 0.01) {
                front.push_back(upper[n]);
            }
            else {
                back.push_back(upper[n]);
            }
        }
        assert(front.size() == 2);
        assert(back.size() == 2);
//        cout << "front: " << front.size() << endl;
//        cout << "back: " << back.size() << endl;
        // change the position if wrong
        if (masses[front[0]].p.y < masses[back[0]].p.y) {
            vector<int> temp = front;
            front = back;
            back = temp;
        }
//
        if (masses[front[0]].p.x > masses[front[1]].p.x) {
            right = front[0];
            left = front[1];
        }
        else {
            right = front[1];
            left = front[0];
        }
        tempCube[5] = right;
        tempCube[6] = left;
        for (int z = 0; z < 4; z++) {
            if(abs(masses[upper[z]].p.x - masses[right].p.x) < 0.01 && masses[upper[z]].p.y < masses[right].p.y - 0.05) {
                tempCube[4] = upper[z];
            }
            if(masses[upper[z]].p.x < masses[right].p.x - 0.05 && masses[upper[z]].p.y < masses[right].p.y -0.05) {
                tempCube[7] = upper[z];
            }
        }
        for (int p = 0; p < 4; p++) {
            if(abs(masses[down[p]].p.x - masses[tempCube[4]].p.x) < 0.01 && abs(masses[down[p]].p.y - masses[tempCube[4]].p.y) < 0.01){
                tempCube[0] = down[p];
            }
            if(abs(masses[down[p]].p.x - masses[tempCube[5]].p.x) < 0.01 && abs(masses[down[p]].p.y - masses[tempCube[5]].p.y) < 0.01){
                tempCube[1] = down[p];
            }
            if(abs(masses[down[p]].p.x - masses[tempCube[6]].p.x) < 0.01 && abs(masses[down[p]].p.y - masses[tempCube[6]].p.y) < 0.01){
                tempCube[2] = down[p];
            }
            if(abs(masses[down[p]].p.x - masses[tempCube[7]].p.x) < 0.01 && abs(masses[down[p]].p.y - masses[tempCube[7]].p.y) < 0.01){
                tempCube[3] = down[p];
            }
        }
//        cout << "tempCube: ";
        for (int j = 0; j < 8; j++) {
            imMasses[8 * i + j] = tempCube[j];
//            cout << tempCube[j] <<" " ;     
        };
        
        for (int k = 0; k < 12; k++) {
            cubeIndices.push_back(tempCube[faceIndices[k][0]]);
            cubeIndices.push_back(tempCube[faceIndices[k][1]]);
            cubeIndices.push_back(tempCube[faceIndices[k][2]]);
//            cubeIndices[36 * i + 1 + 3 * k] = tempCube[faceIndices[k][1]];
//            cubeIndices[36 * i + 2 + 3 * k] = tempCube[faceIndices[k][2]];
        }
        for (int g = 0; g < 12; g++) {
            myEdge_indices.push_back(tempCube[edgeIndices[g][0]]);
            myEdge_indices.push_back(tempCube[edgeIndices[g][1]]);
        }
    }
}


void Robot::updateVertices() {
    for (int i = 0; i < masses.size(); i++) {
        for (int j = 0; j < 3; j++) {
            cubeVertices.push_back(masses[i].p[j]);
//            cubeVertices[3 * i + j] = masses[i].p[j];
        }
    }
    for (int i = 0; i < masses.size(); i++) {
        if(masses[i].p[2] <= 0.11) {
            cubeColor.push_back(0.2);
            cubeColor.push_back(0.2);
            cubeColor.push_back(0.6);
        }
        else{
            cubeColor.push_back(0.8);
            cubeColor.push_back(0.3);
            cubeColor.push_back(0.4);
        }
    }
    for (int i = 0; i < masses.size(); i++) {
        if(masses[i].p[2] <= 0.001) {
            pointColor.push_back(0.1);
            pointColor.push_back(0.9);
            pointColor.push_back(0.1);
        }
        else {
            pointColor.push_back(0.0);
            pointColor.push_back(0.0);
            pointColor.push_back(0.0);
        }
    }
}

void Robot::breathing() {
//    for (int i = 0; i < springs.size(); i++) {
//        // choose 0, 2, 6, 8, 13 as motor cube
//        springs[i].k = gene.k[i];
//    }
//    for (int i = 0; i < 9; i++){
//            for (int j = 0; j < 20; j++) {
//                springs[20 * i + j].b = gene.b[i];
//                springs[20 * i + j].c = gene.c[i];
//            }
//    }
    for (int i = 0; i < 28; i++) {
    springs[i].L0 = springs[i].a + springs[i].b * sin(omega * T + springs[i].c);
    }
}

void Robot::updateSprings() {
    for (int i = 0; i < 9; i++){
            for (int j = 0; j < 20; j++) {
                springs[20 * i + j].b = gene.b[i];
                springs[20 * i + j].c = gene.c[i];
                springs[20 * i + j].k = gene.k[i];
            }
    }
}

void Robot::updateRobot() {
    for (int i = 0; i < masses.size(); i++) {
        masses[i].force = {0, 0, 0};
        glm::dvec3 springForce(0.0);
        glm::dvec3 springDirection(0.0);
        double springLength;
        for (const Spring& spring: springs) {
            // find the corresponding spring
            springLength = glm::distance(masses[spring.m1].p, masses[spring.m2].p);
            if (spring.m1 == i) {
                springDirection = glm::normalize(masses[spring.m2].p - masses[spring.m1].p);
                springForce = springDirection * spring.k * (springLength - spring.L0);
                masses[i].force += springForce;
            }
            else if(spring.m2 == i) {
                springDirection = glm::normalize(masses[spring.m1].p - masses[spring.m2].p);
                springForce = springDirection * spring.k * (springLength - spring.L0);
                masses[i].force += springForce;
            }
        }
        masses[i].force += GRAVITY * MASS;
        if(masses[i].p.z < 0) {
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
//            cout << "friction" << frictionDirection.x << "  " << frictionDirection.y << "  " << frictionDirection.z << " ";
            frictionForce = frictionDirection * F_c * mu;
            masses[i].force += frictionForce;
        }
    }
    for (int i = 0; i < masses.size(); i++) {
    masses[i].a = masses[i].force/MASS;
    masses[i].v += masses[i].a*dt;
    masses[i].p += masses[i].v*dt;
    masses[i].v *= DAMPING;
    }
    //T+= dt;
}

void runningSimulate(Robot& robot, double runningTime) {
    // create a copy
    Robot tempRobot = robot;
    tempRobot.updateSprings();
    double runTime = 0;
    double dtime = 0.001;
    while (runTime < runningTime) {
        tempRobot.breathing();
        tempRobot.updateRobot();
        runTime += dtime;
    }
    tempRobot.setDistance();
    robot.moveDistance = tempRobot.getDistance();
    //std::cout << "distance running: " << robot.moveDistance << endl;
}

bool Robot::theSame(Mass m1, Mass m2){
    if (distance(m1.p, m2.p) < 0.00001){
        return true;
    }
    return false;
}

glm::dvec3 Robot::getPosition() {
    glm::dvec3 pos(0.0);
    for (const Mass& mass: masses) {
        pos += mass.p;
    }
    currentPos = pos/(double)masses.size();
    return currentPos;
}

double Robot::getDistance() {
    return  moveDistance;
}

void Robot::setDistance() {
    currentPos = getPosition();
    moveDistance = glm::distance(currentPos, startPos);
}

vector<Robot> generateRobotGroup(int robotNum) {
    vector<Robot> robotGroup;
    for (int i = 0; i < robotNum; i++) {
        Robot robot(0, 0, 0.01, 20);
        robotGroup.push_back(robot);
    }
    return robotGroup;
}

vector<Robot> crossoverRobot(Robot parent1, Robot parent2) {
    vector<Robot> twoChild;
    Robot child1;
    Robot child2;
    double height1 = 0.0;
    double height2 = 0.0;
    // get the height of each robot
    for (const vector<double>& exist: parent1.existCube) {
        if (exist[2] > height1) height1 = exist[2];
    }
    for (const vector<double>& exist: parent2.existCube) {
        if (exist[2] > height2) height2 = exist[2];
    }
    double height = min(height1, height2);
    // choose a layer
    int randomChoice = rand() % (int)(height/0.1);
    height = 0.1 * double(randomChoice);
    // record the cube need to change
    vector<vector<double>> change1;
    vector<vector<double>> change2;
    // change a layer
    for (const vector<double>& exist: parent1.existCube) {
        if (approximatelyEqual(exist[2], height)) change1.push_back(exist);
    }
    for (int i = 0; i < change1.size(); i++) {
        child1 = deleteCubes(parent1, change1);
        for (const vector<double>& change: change2) {
            if (!child1.checkExist(change[0], change[1], change[2]))
            child1.createCube(change[0], change[1], change[2]);
        }
    }
    for (const vector<double>& exist: parent2.existCube) {
        if (approximatelyEqual(exist[2], height)) change2.push_back(exist);
    }
    for (int i = 0; i < change1.size(); i++) {
        child2 = deleteCubes(parent2, change2);
        for (const vector<double>& change: change1) {
            if (!child2.checkExist(change[0], change[1], change[2]))
            child2.createCube(change[0], change[1], change[2]);
        }
    }
    twoChild.push_back(child1);
    twoChild.push_back(child2);
    return twoChild;
}

Robot mutateRobot(Robot robot) {
    int mutateType = 0;
    cout << "mutate type: " << mutateType << endl;
    // mutate to add a cube
    if (mutateType == 0){
        int mutate = 0;
        while (mutate < 1) {
            int choice = rand() % robot.existCube.size();  // don't delete the motor
            if (canDelete(robot, robot.existCube[choice]) == true) {
                Robot newRobot = deleteCube(robot, robot.existCube[choice][0], robot.existCube[choice][1], robot.existCube[choice][2]);
                mutate++;
                return newRobot;
            }
            else {
                choice = rand() % robot.existCube.size();
            }
        }
        return robot;
    }
    else{
        int mutate = 0;
        while(mutate < 1){
                // choose a cube randomly
                int randomChoiceCube, randomChoiceFace;
                randomChoiceCube = rand() % robot.cubes.size();
        //        cout << "random choose cube: " << randomChoiceCube << endl;
                Cube cube = robot.cubes[randomChoiceCube];
                // choose a face randomly
                randomChoiceFace = rand() % 6;
        //        cout << "random choose face: " << randomChoiceFace << endl;
                // generate a cube in front
                switch (randomChoiceFace) {
                    case 0:
                        // create a front cube
                        if (robot.checkExist(cube.center[0] + 0.1, cube.center[1], cube.center[2])) break;
                        else {
                            robot.createCube(cube.center[0] + 0.1, cube.center[1], cube.center[2]);
                            break;
                        }

                    case 1:
                        if (robot.checkExist(cube.center[0] - 0.1, cube.center[1], cube.center[2])) break;
                        else {
                            robot.createCube(cube.center[0] - 0.1, cube.center[1], cube.center[2]);
                            break;
                        }
                            
                    case 2:
                        if (robot.checkExist(cube.center[0], cube.center[1] + 0.1, cube.center[2])) break;
                        else {
                            robot.createCube(cube.center[0], cube.center[1] + 0.1, cube.center[2]);
                            break;
                        }
            
                    case 3:
                        if (robot.checkExist(cube.center[0], cube.center[1] - 0.1, cube.center[2])) break;
                        else {
                            robot.createCube(cube.center[0], cube.center[1] - 0.1, cube.center[2]);
                            break;
                        }
                        
                    case 4:
                        if (robot.checkExist(cube.center[0], cube.center[1], cube.center[2] + 0.1)) break;
                        else {
                            robot.createCube(cube.center[0], cube.center[1], cube.center[2] + 0.1);
                            break;
                        }
                    case 5:
                        if (robot.checkExist(cube.center[0], cube.center[1], cube.center[2] - 0.1) || cube.center[2] - 0.1<0) break;
                        else {
                            robot.createCube(cube.center[0], cube.center[1], cube.center[2] - 0.1);
                            break;
                        }
                    default:
                        break;
                }
            }
        return robot;
    }
}


vector<Robot> crossoverRobotMotor(Robot parent1, Robot parent2) {
    vector<Robot> result;
    // crossover k
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
    else{
        std::vector<double> tempc = parent1.gene.c;
        parent1.gene.c = parent2.gene.c;
        parent2.gene.c = tempc;
    }
    result.push_back(parent1);
    result.push_back(parent2);
    return result;
}

void selection(vector<Robot>& robotGroup) {
    int popSize = (int)robotGroup.size();
    for(int i = 0; i < popSize; i++) {
       int swaps = 0;         //flag to detect any swap is there or not
       for(int j = 0; j<popSize-i-1; j++) {
          if(robotGroup[j].getDistance() <  robotGroup[j + 1].getDistance()){
             swap(robotGroup[j], robotGroup[j+1]);
             swaps = 1;    //set swap flag
          }
       }
       if(!swaps)
          break;       // No swap in this pass, so array is sorted
    }
    // 0.3 winner
    int winnerNum = popSize * selectPressure;
    vector<Robot> nextGeneration;
    for (int i = 0; i < winnerNum; i++) {
        nextGeneration.push_back(robotGroup[i]);
    }
    
    // crossover best with others 0.2 in total
    int winnerEffects = 0.1 * popSize;
    for (int i = 0; i < winnerEffects; i++) {
        int randomIndex = rand()%robotGroup.size();
        vector<Robot> twoChild;
        twoChild = crossoverRobotMotor(robotGroup[0], robotGroup[randomIndex]);
        nextGeneration.push_back(twoChild[0]);
        nextGeneration.push_back(twoChild[1]);
    }
    
    // 0.4 intotal
    int crossNum = 0.2 * popSize;
    for (int i = 0; i < crossNum; i++) {
        vector<Robot> twoChild;
        int randomIndex1 = rand()%robotGroup.size();
        int randomIndex2 = rand()%robotGroup.size();
        Robot parent1 = robotGroup[randomIndex1];
        Robot parent2 = robotGroup[randomIndex2];
        twoChild = crossoverRobotMotor(parent1, parent2);
        nextGeneration.push_back(twoChild[0]);
        nextGeneration.push_back(twoChild[1]);
    }
    
    // shoud be 0.1 in total
    while (nextGeneration.size() < popSize) {
        Robot robot(0, 0, 0.01, 20);
        robot.setDistance();
        runningSimulate(robot, 2);
        nextGeneration.push_back(robot);
    }
    robotGroup = nextGeneration;
}

double getDiversity(vector<Robot>& robotGroup) {
    double div = 0;
    int n = (int)robotGroup.size();
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            for (int count = 0; count < DIM*DIM; count++) {
                div += abs(robotGroup[i].gene.k[count] - robotGroup[j].gene.k[count])/10000;
            }
            for (int num = 0; num < DIM * DIM; num++) {
                div += abs(robotGroup[i].gene.c[num] - robotGroup[j].gene.c[num]);
                div += abs(robotGroup[i].gene.b[num] - robotGroup[j].gene.b[num]);
            }
        }
    }
    return div;
}

bool canDelete(Robot robot, vector<double> cubePosition) {
    int nearbyNum = 0;
    if(robot.checkExist(cubePosition[0] + 0.1, cubePosition[1], cubePosition[2])) nearbyNum ++;
    if(robot.checkExist(cubePosition[0] - 0.1, cubePosition[1], cubePosition[2])) nearbyNum ++;
    if(robot.checkExist(cubePosition[0], cubePosition[1] + 0.1, cubePosition[2])) nearbyNum ++;
    if(robot.checkExist(cubePosition[0], cubePosition[1] - 0.1, cubePosition[2])) nearbyNum ++;
    if(robot.checkExist(cubePosition[0], cubePosition[1], cubePosition[2] + 0.1)) nearbyNum ++;
    if(robot.checkExist(cubePosition[0], cubePosition[1], cubePosition[2] - 0.1)) nearbyNum ++;
    if (nearbyNum > 3 || nearbyNum == 1) {
        return true;
    }
    else {
        return false;
    }
}


bool approximatelyEqual(double a, double b)
{
    return fabs(a - b) <= ( (fabs(a) < fabs(b) ? fabs(b) : fabs(a)) * 0.01);
}

Robot deleteCubes(Robot robot, vector<vector<double>> positions) {
    vector<vector<double>> newExist;
    for (const vector<double>& cube: robot.existCube) {
        bool exist = false;
        for (const vector<double>& position: positions) {
            if(approximatelyEqual(cube[0], position[0]) && approximatelyEqual(cube[1], position[1]) && approximatelyEqual(cube[2], position[2])) {
                exist = true;
            }
        }
        if (exist == false) {
            newExist.push_back(cube);
        }
        cout << newExist.size() << endl;
    }
    Robot newRobot(newExist);
    return newRobot;
}

Robot deleteCube(Robot robot, double x, double y, double z) {
    int index = 0;
    for (const vector<double>& cube: robot.existCube) {
        if (approximatelyEqual(cube[0], x) && approximatelyEqual(cube[1], y) && approximatelyEqual(cube[2], z)) {
            break;
        }
        else {
            index ++;
        }
    }
    robot.existCube.erase(robot.existCube.begin() + index);
    cout << robot.existCube.size() << endl;
    Robot newRobot(robot.existCube);
    return newRobot;
}
vector<Robot> geneticAlgorithm(int robotCount, int generationNum, int robotReturn, double cycleTime, bool record) {
    vector<Robot> robotGroup = generateRobotGroup(robotCount);
    vector<Robot> returnRobot;
    ofstream dotchart;
    ofstream Diversity;
    ofstream learningCurve;
    learningCurve.open("/Users/jiaweichen/Desktop/learning_curve.txt");
    dotchart.open("/Users/jiaweichen/Desktop/dot_chart.txt");
    Diversity.open("/Users/jiaweichen/Desktop/diversity.txt");
    
    double diversity;
    for (int i = 0; i < generationNum; i++) {
        cout << "-----------------generation:"  << i << "----------------" << endl;
        diversity = getDiversity(robotGroup);
        if(record){
            Diversity << diversity << endl;
        }
        // update the running
        for (Robot& robot: robotGroup) {
            runningSimulate(robot, cycleTime);
        }
        selection(robotGroup);
        for (Robot& robot: robotGroup) {
            cout << "after selection: " << robot.getDistance() << endl;
            if(record){
                dotchart << robot.getDistance() << ' ';
            }
        }
        dotchart << endl;
        learningCurve << robotGroup[0].getDistance() << endl;
    }
    for (int i = 0; i < robotReturn; i++ ) {
        returnRobot.push_back(robotGroup[i]);
    }
    dotchart.close();
    Diversity.close();
    learningCurve.close();
    return returnRobot;
}


