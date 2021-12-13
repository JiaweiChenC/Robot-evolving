#include "robot.hpp"
#include <random>

Robot::Robot(double x, double y, double z, int robotSize) {
    int randomChoiceCube;
    int randomChoiceFace;
    // cout << x << " " << y << " " << z << " " << endl;
    createCube(x, y, z);
    //    cout << "the first cube" << endl;
    while (cube_num < robotSize) {
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
                if (checkExist(cube.center[0] + 0.1, cube.center[1], cube.center[2]))
                    break;
                else {
                    createCube(cube.center[0] + 0.1, cube.center[1], cube.center[2]);
                    break;
                }

            case 1:
                if (checkExist(cube.center[0] - 0.1, cube.center[1], cube.center[2]))
                    break;
                else {
                    createCube(cube.center[0] - 0.1, cube.center[1], cube.center[2]);
                    break;
                }

            case 2:
                if (checkExist(cube.center[0], cube.center[1] + 0.1, cube.center[2]))
                    break;
                else {
                    createCube(cube.center[0], cube.center[1] + 0.1, cube.center[2]);
                    break;
                }

            case 3:
                if (checkExist(cube.center[0], cube.center[1] - 0.1, cube.center[2]))
                    break;
                else {
                    createCube(cube.center[0], cube.center[1] - 0.1, cube.center[2]);
                    break;
                }

            case 4:
                if (checkExist(cube.center[0], cube.center[1], cube.center[2] + 0.1))
                    break;
                else {
                    createCube(cube.center[0], cube.center[1], cube.center[2] + 0.1);
                    break;
                }
            case 5:
                if (checkExist(cube.center[0], cube.center[1], cube.center[2] - 0.1) || cube.center[2] - 0.1 < 0)
                    break;
                else {
                    createCube(cube.center[0], cube.center[1], cube.center[2] - 0.1);
                    break;
                }

            default:
                break;
        }
    }

    if (EVOLUTION) {
        for (int count = 0; count < 28; count++) {
            gene.k.push_back(dist1(rng));
        }
        // have motorCube to generate power for cube
        for (int count = 0; count < 28; count++) {
            gene.c.push_back(dist0(rng) * 2 * M_PI);
            gene.b.push_back(dist2(rng));
        }
    }
}

Robot::Robot(std::vector<std::vector<double>> cubes) {
    for (const std::vector<double>& cube : cubes) {
        createCube(cube[0], cube[1], cube[2]);
    }
}

bool Robot::checkExist(double x, double y, double z) {
    bool exist = false;
    for (const auto& position : existCube) {
        if ((abs(position[0] - x) < 0.001 && abs(position[1] - y) < 0.001 && abs(position[2] - z) < 0.001)) {
            exist = true;
        }
    }
    return exist;
}

void Robot::createCube(double x, double y, double z) {
    Cube cube;
    //  this center is a center of xy face
    cube.center = {x, y, z};
    cube_num += 1;
    std::vector<double> position = {x, y, z};
    existCube.push_back(position);
    int n = (int)masses.size();
    // int n2 = (int)springs.size();
    // first create 8 masses
    std::vector<Mass> tempMasses(8);
    tempMasses[0] = {MASS, {x + LENGTH / 2, y + LENGTH / 2, z}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    tempMasses[1] = {MASS, {x + LENGTH / 2, y - LENGTH / 2, z}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    tempMasses[2] = {MASS, {x - LENGTH / 2, y - LENGTH / 2, z}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    tempMasses[3] = {MASS, {x - LENGTH / 2, y + LENGTH / 2, z}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    tempMasses[4] = {MASS, {x + LENGTH / 2, y + LENGTH / 2, z + LENGTH}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    tempMasses[5] = {MASS, {x + LENGTH / 2, y - LENGTH / 2, z + LENGTH}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    tempMasses[6] = {MASS, {x - LENGTH / 2, y - LENGTH / 2, z + LENGTH}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    tempMasses[7] = {MASS, {x - LENGTH / 2, y + LENGTH / 2, z + LENGTH}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

    // check the mass created if coinciding with the previous cube if is
    // make m = 0 coinciding pushback the mass overlap, imMasses pushback
    // record all the indices
    std::vector<int> coinciding;
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
        if (tempMasses[i].m != 0) {
            masses.push_back(tempMasses[i]);
            imMasses.push_back(n + count);
            cube.cubeMass.push_back(n + count);
            count++;
        }
    }

    // create springs, this is for the nonconinciding masses
    for (int i = n; i < masses.size() - 1; i++) {
        for (int j = i + 1; j < masses.size(); j++) {
            Spring s;
            s.k = SPRING_CONSTANT;
            s.m1 = j;
            s.m2 = i;
            springs.push_back(s);
        }
    }

    // create springs, this is for the coinciding masses
    for (int i = 0; i < coinciding.size(); i++) {
        for (int j = n; j < masses.size(); j++) {
            Spring s;
            s.k = SPRING_CONSTANT;
            s.m1 = coinciding[i];
            s.m2 = j;
            springs.push_back(s);
        }
    }
    for (Spring& spring : springs) {
        spring.L0 = distance(masses[spring.m1].p, masses[spring.m2].p);
        spring.a = spring.L0;
    }
    cubes.push_back(cube);
}

bool Robot::theSame(Mass m1, Mass m2) {
    if (distance(m1.p, m2.p) < 0.00001) {
        return true;
    }
    return false;
}

int getDiversity(std::vector<Robot>& robotGroup) {
    int div = 0;
    int n = (int)robotGroup.size();
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            for (const std::vector<double>& position : robotGroup[i].existCube) {
                if (!robotGroup[j].checkExist(position[0], position[1], position[2])) {
                    div++;
                }
            }
        }
    }
    return div;
}

std::vector<Robot> generateRobotGroup(int robotNum) {
    std::vector<Robot> robotGroup;
    for (int i = 0; i < robotNum; i++) {
        Robot robot(0, 0, 0.0, 20);
        robotGroup.push_back(robot);
    }
    return robotGroup;
}

std::vector<Robot> crossoverRobot(Robot parent1, Robot parent2) {
    std::vector<Robot> twoChild;
    Robot child1;
    Robot child2;
    double height1 = 0.0;
    double height2 = 0.0;
    // get the height of parents
    for (const std::vector<double>& exist : parent1.existCube) {
        if (exist[2] > height1) height1 = exist[2];
    }
    for (const std::vector<double>& exist : parent2.existCube) {
        if (exist[2] > height2) height2 = exist[2];
    }
    // exchangable height
    double height = std::min(height1, height2);
    // choose a layer, +0.1 to avoid rand() % 0;
    int randomChoice = rand() % (int)((height + 0.1) / 0.1) - 1;
    height = 0.1 * double(randomChoice);
    // record the cube need to exchange
    std::vector<std::vector<double>> change1;
    std::vector<std::vector<double>> change2;
    // all the cubes in the choosen height
    for (const std::vector<double>& exist : parent1.existCube) {
        if (approximatelyEqual(exist[2], height)) change1.push_back(exist);
    }
    for (const std::vector<double>& exist : parent2.existCube) {
        if (approximatelyEqual(exist[2], height)) change2.push_back(exist);
    }

    child1 = deleteCubes(parent1, change1);
    for (const std::vector<double>& change : change2) {
        //            if (!child1.checkExist(change[0], change[1], change[2]))
        child1.createCube(change[0], change[1], change[2]);
    }

    child2 = deleteCubes(parent2, change2);
    for (const std::vector<double>& change : change1) {
        //            if (!child2.checkExist(change[0], change[1], change[2]))
        child2.createCube(change[0], change[1], change[2]);
    }
    twoChild.push_back(child1);
    twoChild.push_back(child2);
    return twoChild;
}

Robot mutateRobot(Robot robot) {
    int mutateType = rand() % 2;
    if (robot.cubes.size() > 40) {
        mutateType = 0;
    } else if (robot.cubes.size() < 15) {
        mutateType = 1;
    } else {
        mutateType = mutateType;
    }
    // cout << "mutate type: " << mutateType << endl;
    // mutate to add a cube
    if (mutateType == 0) {
        int mutate1 = 0;
        int times = 0;
        while (mutate1 < 1) {
            int choice = rand() % robot.existCube.size();
            if (canDelete(robot, robot.existCube[choice]) == true) {
                Robot newRobot = deleteCube(robot, robot.existCube[choice][0], robot.existCube[choice][1], robot.existCube[choice][2]);
                mutate1++;
                return newRobot;
            } else {
                choice = rand() % robot.existCube.size();
                times++;
                // cout << "times: " << time << endl;
                if (times > 4) {
                    mutateType = 1;
                    mutate1 = 1;
                }
            }
        }
        return robot;
    } else {
        int mutate2 = 0;
        while (mutate2 < 1) {
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
                    if (robot.checkExist(cube.center[0] + 0.1, cube.center[1], cube.center[2]))
                        break;
                    else {
                        robot.createCube(cube.center[0] + 0.1, cube.center[1], cube.center[2]);
                        mutate2++;
                        break;
                    }

                case 1:
                    if (robot.checkExist(cube.center[0] - 0.1, cube.center[1], cube.center[2]))
                        break;
                    else {
                        robot.createCube(cube.center[0] - 0.1, cube.center[1], cube.center[2]);
                        mutate2++;
                        break;
                    }

                case 2:
                    if (robot.checkExist(cube.center[0], cube.center[1] + 0.1, cube.center[2]))
                        break;
                    else {
                        robot.createCube(cube.center[0], cube.center[1] + 0.1, cube.center[2]);
                        mutate2++;
                        break;
                    }

                case 3:
                    if (robot.checkExist(cube.center[0], cube.center[1] - 0.1, cube.center[2]))
                        break;
                    else {
                        robot.createCube(cube.center[0], cube.center[1] - 0.1, cube.center[2]);
                        mutate2++;
                        break;
                    }

                case 4:
                    if (robot.checkExist(cube.center[0], cube.center[1], cube.center[2] + 0.1))
                        break;
                    else {
                        robot.createCube(cube.center[0], cube.center[1], cube.center[2] + 0.1);
                        mutate2++;
                        break;
                    }
                case 5:
                    if (robot.checkExist(cube.center[0], cube.center[1], cube.center[2] - 0.1) || cube.center[2] - 0.05 < 0)
                        break;
                    else {
                        robot.createCube(cube.center[0], cube.center[1], cube.center[2] - 0.1);
                        mutate2++;
                        break;
                    }
                default:
                    break;
            }
        }
        return robot;
    }
}

// just a buble ranking
void selection(std::vector<Robot>& robotGroup) {
    int popSize = (int)robotGroup.size();
    // sorting the whole population
    for (int i = 0; i < popSize; i++) {
        int swaps = 0;  // flag to detect any swap is there or not
        for (int j = 0; j < popSize - i - 1; j++) {
            if (robotGroup[j].moveDistance < robotGroup[j + 1].moveDistance) {
                std::swap(robotGroup[j], robotGroup[j + 1]);
                swaps = 1;  // set swap flag
            }
        }
        if (!swaps)
            break;  // No swap in this pass, so array is sorted
    }
}

void crossover(std::vector<Robot>& robotGroup) {
    int popSize = (int)robotGroup.size();
    // 0.4 winners will go to next generation
    int winnerNum = popSize * SELECT_PRESSURE;
    std::vector<Robot> nextGeneration;
    for (int i = 0; i < winnerNum; i++) {
        nextGeneration.push_back(robotGroup[i]);
    }

    // crossover best with others 0.1 in total
    int winnerEffects = 0.05 * popSize;
    for (int i = 0; i < winnerEffects; i++) {
        int randomIndex = rand() % popSize;
        std::vector<Robot> twoChild;
        twoChild = crossoverRobot(robotGroup[0], robotGroup[randomIndex]);
        nextGeneration.push_back(twoChild[0]);
        nextGeneration.push_back(twoChild[1]);
    }

    // 0.4 intotal
    int crossNum = 0.2 * popSize;
    for (int i = 0; i < crossNum; i++) {
        std::vector<Robot> twoChild;
        int randomIndex1 = rand() % robotGroup.size();
        int randomIndex2 = rand() % robotGroup.size();
        Robot parent1 = robotGroup[randomIndex1];
        Robot parent2 = robotGroup[randomIndex2];
        twoChild = crossoverRobot(parent1, parent2);
        nextGeneration.push_back(twoChild[0]);
        nextGeneration.push_back(twoChild[1]);
    }

    // generate some new random robot
    while (nextGeneration.size() < popSize) {
        Robot robot(0, 0, 0, 20);
        nextGeneration.push_back(robot);
    }
    robotGroup = nextGeneration;
}

bool canDelete(Robot robot, std::vector<double> cubePosition) {
    int nearbyNum = 0;
    if (robot.checkExist(cubePosition[0] + 0.1, cubePosition[1], cubePosition[2])) nearbyNum++;
    if (robot.checkExist(cubePosition[0] - 0.1, cubePosition[1], cubePosition[2])) nearbyNum++;
    if (robot.checkExist(cubePosition[0], cubePosition[1] + 0.1, cubePosition[2])) nearbyNum++;
    if (robot.checkExist(cubePosition[0], cubePosition[1] - 0.1, cubePosition[2])) nearbyNum++;
    if (robot.checkExist(cubePosition[0], cubePosition[1], cubePosition[2] + 0.1)) nearbyNum++;
    if (robot.checkExist(cubePosition[0], cubePosition[1], cubePosition[2] - 0.1)) nearbyNum++;
    if (nearbyNum > 3 || nearbyNum == 1) {
        return true;
    } else {
        return false;
    }
}

bool approximatelyEqual(double a, double b) {
    return fabs(a - b) <= ((fabs(a) < fabs(b) ? fabs(b) : fabs(a)) * 0.01);
}

Robot deleteCubes(Robot robot, std::vector<std::vector<double>> positions) {
    std::vector<std::vector<double>> newExist;
    for (const std::vector<double>& cube : robot.existCube) {
        bool exist = false;
        for (const std::vector<double>& position : positions) {
            if (approximatelyEqual(cube[0], position[0]) && approximatelyEqual(cube[1], position[1]) && approximatelyEqual(cube[2], position[2])) {
                exist = true;
            }
        }
        if (exist == false) {
            newExist.push_back(cube);
        }
    }
    Robot newRobot(newExist);
    return newRobot;
}

Robot deleteCube(Robot robot, double x, double y, double z) {
    int index = 0;
    for (const std::vector<double>& cube : robot.existCube) {
        if (approximatelyEqual(cube[0], x) && approximatelyEqual(cube[1], y) && approximatelyEqual(cube[2], z)) {
            break;
        } else {
            index++;
            if (index == robot.cubes.size()) {
                break;
            }
        }
    }
    robot.existCube.erase(robot.existCube.begin() + index);
    //    cout << robot.existCube.size() << endl;
    Robot newRobot(robot.existCube);
    return newRobot;
}
