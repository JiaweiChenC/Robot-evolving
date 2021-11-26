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
Robot mutateRobot(Robot robot);
vector<Robot> crossoverRobot();
double getDiversity(vector<Robot> robotGroup);
double getGroupDistance(vector<Robot> robot);
#endif /* ea_hpp */
