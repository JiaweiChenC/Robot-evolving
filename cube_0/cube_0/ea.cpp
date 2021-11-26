//
//  ea.cpp
//  cube_0
//
//  Created by Jiawei Chen on 11/26/21.
//

#include "ea.hpp"
#include "Cube.hpp"
// generate a robot group
vector<Robot> generateRobotGroup(int robotNum) {
    vector<Robot> robotGroup;
    for (int i = 0; i < robotNum; i++) {
        Robot robot(0, 0, 0.1);
        robotGroup.push_back(robot);
    }
    return robotGroup;
}

Robot mutateRobot(Robot robot) {
    
}
vector<Robot> crossoverRobot();
double getDiversity(vector<Robot> robotGroup);
double getGroupDistance(vector<Robot> robot);
