#include "Cube.hpp"
const double MASS = 0.1;
double LENGTH = 0.1;
double gravity = 9.81;
double T = 0;
double L0 = 0.1;
double dt = 0.001;
double mu = 0.8;
int DIM = 3;
double omega = 1;
double restoreConstant = 10000;
double kGround = 5000;
int springConstant = 5000;
int motorCube = 5;
double DAMPING = 1;
const double mutatePro = 0.1;
const double crossPro = 0.4;
const double selectPressure = 0.6;
bool evolution = true;
glm::dvec3 GRAVITY = {0, 0, -9.8};
extern GLdouble cubeVertices[]; // cube_count * 8 points * 3 (x, y, z)
extern GLdouble cubeColor[]; // cube_count * 8 points * 3 bit
extern GLuint cubeIndices[];   // cube_count * 6 faces * 2 triangles * 3 indices
extern GLdouble myEdge_color[];      // cube_count * 8 points * 3 bit
extern GLuint myEdge_indices[]; // cube_count * 12 * 2
extern GLdouble myShade_vertex[];
extern GLdouble myShade_color[];
extern GLuint myShadeindices[];
extern vector<vector<GLuint>> faceIndices;
extern vector<vector<GLuint>> edgeIndices;

std::random_device dev;
std::mt19937 rng(dev());
std::uniform_real_distribution<> dist0(0, 1);
std::uniform_int_distribution<> dist1(3000, 10000);
std::uniform_real_distribution<> dist2(-0.4, 0.4);
using namespace std;
Robot::Robot(double x, double y, double z){
    for (int i = 0; i < DIM; i++){
        for (int j = 0; j < DIM; j++){
            for (int k = 0; k < DIM; k++){
                createCube((double)(k)/10.0 + x, (double)(j)/10.0 + y, (double)(i)/10.0 + z);
                if (evolution) {
                    // get the correspond parameters
                    // every cube will have a k
                    for (int count = 0; count < cube_num; count++) {
                        gene.k.push_back(dist1(rng));
                    }
                    // have motorCube to generate power for cube
                    for (int count = 0; count < motorCube; count++) {
                        gene.c.push_back(dist0(rng) * 2 * M_PI);
                        gene.b.push_back(dist2(rng));
                    }
                }
            }
        }
    }
    startPos = getPosition();
}

void Robot::createCube (double x, double y, double z) {
    int n = (int)masses.size();
    // int n2 = (int)springs.size();
    // first create 8 masses
    vector<Mass> tempMasses(8);
    tempMasses[0] = {MASS,{x+LENGTH/2,y+LENGTH/2,z},{0,0,0},{0,0,0}};
    tempMasses[1] = {MASS,{x+LENGTH/2,y-LENGTH/2,z},{0,0,0},{0,0,0}};
    tempMasses[2] = {MASS,{x-LENGTH/2,y-LENGTH/2,z},{0,0,0},{0,0,0}};
    tempMasses[3] = {MASS,{x-LENGTH/2,y+LENGTH/2,z},{0,0,0},{0,0,0}};
    tempMasses[4] = {MASS,{x+LENGTH/2,y+LENGTH/2,z+LENGTH},{0,0,0},{0,0,0}};
    tempMasses[5] = {MASS,{x+LENGTH/2,y-LENGTH/2,z+LENGTH},{0,0,0},{0,0,0}};
    tempMasses[6] = {MASS,{x-LENGTH/2,y-LENGTH/2,z+LENGTH},{0,0,0},{0,0,0}};
    tempMasses[7] = {MASS,{x-LENGTH/2,y+LENGTH/2,z+LENGTH},{0,0,0},{0,0,0}};
    
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
}

void Robot::someStuffToMakesuretheDrawingWroking() {
    for (int i = 0; i < DIM*DIM*DIM; i++) {
        vector<int> tempCube(8);
        vector<int> upper(0);
        vector<int> down(0);
        vector<int> front(0);
        vector<int> back(0);
        int right;
        int left;
        for (int j = 0; j < 8; j++) {
            tempCube[j] = imMasses[8 * i + j];
        }
        for (int n = 0; n < 8; n++) {
            if (masses[tempCube[n]].p.z == masses[tempCube[0]].p.z){
                upper.push_back(tempCube[n]);
            }
            else {
                down.push_back(tempCube[n]);
            }
        }
        cout << upper[0]<<  endl;
        // change position if wrong
        if (masses[upper[0]].p.z < masses[down[0]].p.z) {
            vector<int> temp = upper;
            upper = down;
            down = temp;
        }
        
        for (int n = 0; n < 4; n++) {
            if (masses[upper[n]].p.y == masses[upper[0]].p.y) {
                front.push_back(upper[n]);
            }
            else {
                back.push_back(upper[n]);
            }
        }
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
            if(masses[upper[z]].p.x == masses[right].p.x && masses[upper[z]].p.y < masses[right].p.y) {
                tempCube[4] = upper[z];
            }
            if(masses[upper[z]].p.x < masses[right].p.x && masses[upper[z]].p.y < masses[right].p.y) {
                tempCube[7] = upper[z];
            }
        }
        for (int p = 0; p < 4; p++) {
            if(masses[down[p]].p.x == masses[tempCube[4]].p.x && masses[down[p]].p.y == masses[tempCube[4]].p.y){
                tempCube[0] = down[p];
            }
            if(masses[down[p]].p.x == masses[tempCube[5]].p.x && masses[down[p]].p.y == masses[tempCube[5]].p.y){
                tempCube[1] = down[p];
            }
            if(masses[down[p]].p.x == masses[tempCube[6]].p.x && masses[down[p]].p.y == masses[tempCube[6]].p.y){
                tempCube[2] = down[p];
            }
            if(masses[down[p]].p.x == masses[tempCube[7]].p.x && masses[down[p]].p.y == masses[tempCube[7]].p.y){
                tempCube[3] = down[p];
            }
        }
        for (int j = 0; j < 8; j++) {
            imMasses[8 * i + j] = tempCube[j];
        };
        
        for (int k = 0; k < 12; k++) {
            cubeIndices[36 * i + 3 * k] = tempCube[faceIndices[k][0]];
            cubeIndices[36 * i + 1 + 3 * k] = tempCube[faceIndices[k][1]];
            cubeIndices[36 * i + 2 + 3 * k] = tempCube[faceIndices[k][2]];
        }
        for (int g = 0; g < 12; g++) {
            myEdge_indices[24 * i + 2 * g] = tempCube[edgeIndices[g][0]];
            myEdge_indices[24 * i + 2 * g + 1] = tempCube[edgeIndices[g][1]];
        }
    }
}

void Robot::updateVertices() {
    for (int i = 0; i < masses.size(); i++) {
        for (int j = 0; j < 3; j++) {
            cubeVertices[3 * i + j] = masses[i].p[j];
        }
    }
}

void Robot::breathing() {
    for (int i = 0; i < springs.size(); i++) {
        // choose 0, 2, 6, 8, 13 as motor cube
        if (i  < 180) {
            springs[i].L0 = springs[i].a + springs[i].b * sin(10 * T + springs[i].c);
            cout <<"springLength: " << springs[i].L0 << endl;
            cout << "a: " << springs[i].a << endl;
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
            double F_c = -kGround * masses[i].p.z;
            masses[i].force.z += F_c;
            glm::dvec3 frictionForce(0.0);
            glm::dvec3 frictionDirection(0.0);
            frictionDirection = glm::normalize(glm::dvec3(-masses[i].v.x, -masses[i].v.y, 0));
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
    T+= dt;
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
    moveDistance = glm::distance(currentPos, startPos);
    return  moveDistance;
}

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
