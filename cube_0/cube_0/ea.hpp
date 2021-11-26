//
//  ea.hpp
//  cube_0
//
//  Created by Jiawei Chen on 11/26/21.
//

#ifndef ea_hpp
#define ea_hpp
#include "Cube.hpp"

#include <stdio.h>

vector<Robot> generateRobotGroup(int robotNum);
void mutateRobot(Robot& robot);
void selection(vector<Robot>& robotGroup);
void crossoverRobot(vector<Robot>& robotGroup);
double getDiversity(vector<Robot>& robotGroup);
double getGroupDistance(vector<Robot> robot);
vector<Robot> geneticAlgorithm(int robotCount, int generationNum, int robotReturn, double cycleTime);
#endif /* ea_hpp */
