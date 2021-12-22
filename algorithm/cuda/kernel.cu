#include "kernel.cuh"
#include "stdio.h"
#include "config.hpp"
#include "math_constants.h"
#include <assert.h>
#include "device_launch_parameters.h"


// check if two number is equal
inline __device__ bool approximatelyEqual(double a, double b) {
    return fabs(a - b) <= ((fabs(a) < fabs(b) ? fabs(b) : fabs(a)) * 0.01);
}

// generate a random number between 0, 1;
 __device__ double generate(curandState* globalState, int ind)
 {
     curandState localState = globalState[ind];
     double random = curand_uniform(&localState);
     globalState[ind] = localState;
     return random;
 }

// get the position of robot
inline __device__ dvec2 knl_getRobotPosition(const knl_Robot& robot) {
    unsigned int N = robot.num_masses;
    double x = 0, y = 0;
    for (unsigned int i = 0; i < N; ++ i) {
        x += robot.masses[i].p.x;
        y += robot.masses[i].p.y;
    }
    return { x / N, y / N };
}

// calculate the 2D distance
inline __device__ double knl_distance(const dvec3& p1, const dvec3& p2) {
    const double dx = p1.x - p2.x, dy = p1.y - p2.y, dz = p1.z - p2.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

// calculate the 3D distance
inline __device__ double knl_distance(const dvec2& p1, const dvec2& p2) {
    const double dx = p1.x - p2.x, dy = p1.y - p2.y;
    return sqrt(dx * dx + dy * dy);
}

inline __device__ dvec3 knl_subtract(const dvec3& p1, const dvec3& p2) {
    return { p1.x - p2.x, p1.y - p2.y, p1.z - p2.z };
}

inline __device__ dvec3 knl_add(const dvec3& p1, const dvec3& p2) {
    return {p1.x + p2.x, p1.y + p2.y, p1.z + p2.z};
}

inline __device__ void knl_add_to(dvec3& target, const dvec3& src) {
    target.x += src.x;
    target.y += src.y;
    target.z += src.z;
}

inline __device__ void knl_add_to(dvec3& target, double src) {
    target.x += src;
    target.y += src;
    target.z += src;
}

inline __device__ dvec3 knl_multiply(const dvec3& p1, const dvec3& p2) {
    return { p1.x * p2.x, p1.y * p2.y, p1.z * p2.z };
}

inline __device__ dvec3 knl_multiply(const dvec3& p1, double p2) {
    return { p1.x * p2, p1.y * p2, p1.z * p2 };
}

inline __device__ void knl_multiply_to(dvec3& p1, double p2) {
    p1.x *= p2;
    p1.y *= p2;
    p1.z *= p2;
}


// normalize a vector, used for get the direction of force
inline __device__ dvec3 knl_normalize(const dvec3& p1) {
    const double magnitude = sqrt(p1.x * p1.x + p1.y * p1.y + p1.z * p1.z);
    if (magnitude < 1e-6) {
        return { 0, 0, 0 };
    }
    return { p1.x / magnitude, p1.y / magnitude, p1.z / magnitude };
}


// if two mass is the same
inline __device__ bool theSame(knl_Mass m1, knl_Mass m2) {
    if (knl_distance(m1.p, m2.p) < 0.0001) {
        return true;
    }
    return false;
}

// update the robot every time step using Newton's law
inline __device__ void knl_updateRobot(knl_Robot& robot) {
    unsigned int N = robot.num_masses;
    unsigned int M = robot.num_springs;

    for (unsigned int i = 0; i < N; ++ i) {
        auto& mass = robot.masses[i];
        mass.force.x = mass.force.y = mass.force.z = 0;
        for (unsigned int j = 0; j < M; ++ j) {
            const auto& spring = robot.springs[j];
            if (spring.m1 == i) {
                auto springLength = knl_distance(robot.masses[spring.m1].p, robot.masses[spring.m2].p);
                auto springDirection = knl_normalize(knl_subtract(robot.masses[spring.m2].p, robot.masses[spring.m1].p));
                auto springForce = knl_multiply(springDirection, spring.k * (springLength - spring.L0));
                knl_add_to(mass.force, springForce);
            } else if (spring.m2 == i) {
                auto springLength = knl_distance(robot.masses[spring.m1].p, robot.masses[spring.m2].p);
                auto springDirection = knl_normalize(knl_subtract(robot.masses[spring.m1].p, robot.masses[spring.m2].p));
                auto springForce = knl_multiply(springDirection, spring.k * (springLength - spring.L0));
                knl_add_to(mass.force, springForce);
            }
        }
        mass.force.z -= GRAVITY * MASS;
        if (mass.p.z < 0) {
            dvec3 frictionDirection;
            // static friction
            if (mass.v.x == 0 && mass.v.y == 0) {
                if (mass.force.x == 0 && mass.force.y == 0) {
                    frictionDirection = { 0, 0, 0 };
                } else {
                    frictionDirection = knl_normalize({ -mass.force.x, -mass.force.y, 0 });
                }
            }
            // move friction
            else {
                frictionDirection = knl_normalize({ -mass.v.x, -mass.v.y, 0 });
            }
            double F_c = -K_GROUND * mass.p.z;
            mass.force.z += F_c;
            dvec3 frictionForce = knl_multiply(frictionDirection, F_c * MU);
            knl_add_to(mass.force, frictionForce);
        }
    }
    
    for (unsigned int i = 0; i < N; ++ i) {
        auto& mass = robot.masses[i];
        mass.a.x = mass.force.x / MASS;
        mass.a.y = mass.force.y / MASS;
        mass.a.z = mass.force.z / MASS;
        
        knl_add_to(mass.v, knl_multiply(mass.a, DT));
        knl_add_to(mass.p, knl_multiply(mass.v, DT));
        knl_multiply_to(mass.v, DAMPING);
    }
}

// create a robot using a list of positions
__device__ knl_Robot create_robot(dvec3* cubes, int N)
{   knl_Robot robot;
    for (int i = 0; i < N; i++) {
        dvec3 cube;
        cube = cubes[i];
        robot.create_cube(cube.x, cube.y, cube.z);
        }
    return robot;
}

// if the cube already exist in the robot, use to avid the coinciding
inline __device__ bool knl_Robot::check_exist(double x, double y, double z)
{
    bool exist = false;
    dvec3 position;
    for (int i = 0; i < num_cubes; i++)
    {
        position = existCubes[i];
        if ((fabs(position.x - x) < 0.001 && fabs(position.y - y) < 0.001 && fabs(position.z - z) < 0.001))
        {
            exist = true;
        }
    }
    return exist;
}

// create a cube at position x, y, z
inline __device__ void knl_Robot::create_cube(double x, double y, double z) {
    existCubes[num_cubes] = {x, y, z};
    num_cubes += 1;

    int  n = num_masses;

    // first create 8 masses
    struct knl_Mass tempMasses[8];
    tempMasses[0] = {MASS, {x + LENGTH / 2, y + LENGTH / 2, z}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    tempMasses[1] = {MASS, {x + LENGTH / 2, y - LENGTH / 2, z}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    tempMasses[2] = {MASS, {x - LENGTH / 2, y - LENGTH / 2, z}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    tempMasses[3] = {MASS, {x - LENGTH / 2, y + LENGTH / 2, z}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    tempMasses[4] = {MASS, {x + LENGTH / 2, y + LENGTH / 2, z + LENGTH}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    tempMasses[5] = {MASS, {x + LENGTH / 2, y - LENGTH / 2, z + LENGTH}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    tempMasses[6] = {MASS, {x - LENGTH / 2, y - LENGTH / 2, z + LENGTH}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    tempMasses[7] = {MASS, {x - LENGTH / 2, y + LENGTH / 2, z + LENGTH}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

    // rewrite all the stuff without vector
    int coin_size = 0;
    int coinciding[8] = {-1};
    for (int j = 0; j < 8; j++) {
        for (int i = 0; i < n; i++) {
            if (theSame(tempMasses[j], knl_masses[i])) {
                coinciding[coin_size] = i;
                ++ coin_size;
                tempMasses[j].m = 0;
            }
        }
    }

    num_masses += 8 - coin_size;
    // pushback not coinciding masses to masses ( array version )
    int mass_count = 0;
    for (int i = 0; i < 8; i++) {
        if (tempMasses[i].m != 0) {
            knl_masses[n + mass_count] = tempMasses[i];
            ++ mass_count;
        }
    }
    // create springs, this is for the nonconinciding masses
    for (int i = n; i < num_masses - 1; i++) {
        for (int j = i + 1; j < num_masses; j++) {
            knl_Spring s;
            s.k = SPRING_CONSTANT;
            s.m1 = j;
            s.m2 = i;
            knl_springs[num_springs] = s;
            ++ num_springs;
        }
    }

    // create springs, this is for the coinciding masses
    for (int i = 0; i < coin_size; i++) {
        for (int j = n; j < num_masses; j++) {
            knl_Spring s;
            s.k = SPRING_CONSTANT;
            s.m1 = coinciding[i];
            s.m2 = j;
            knl_springs[num_springs] = s;
            ++ num_springs;
        }
    }

    for (int i = 0; i < num_springs; i++ ) {
        knl_springs[i].L0 = knl_distance(knl_masses[knl_springs[i].m1].p, knl_masses[knl_springs[i].m2].p);
        knl_springs[i].a = knl_springs[i].L0;
    }
}


//check if we can delete this cube
__device__ bool canDelete(knl_Robot robot, dvec3 cubePosition)
{
    int nearbyNum = 0;
    if (robot.check_exist(cubePosition.x + 0.1, cubePosition.y, cubePosition.z))
        nearbyNum++;
    if (robot.check_exist(cubePosition.x - 0.1, cubePosition.y, cubePosition.z))
        nearbyNum++;
    if (robot.check_exist(cubePosition.x, cubePosition.y + 0.1, cubePosition.z))
        nearbyNum++;
    if (robot.check_exist(cubePosition.x, cubePosition.y - 0.1, cubePosition.z))
        nearbyNum++;
    if (robot.check_exist(cubePosition.x, cubePosition.y, cubePosition.z + 0.1))
        nearbyNum++;
    if (robot.check_exist(cubePosition.x, cubePosition.y, cubePosition.z - 0.1))
        nearbyNum++;
    if (nearbyNum > 3 || nearbyNum == 1)
    {
        return true;
    }
    else
    {
        return false;
    }
}

// delete a bunch of cubes, this is for crossover
inline __device__ void deleteCubes(knl_Robot& robot, dvec3* positions, int N)
{
    dvec3* newExist;
    newExist = new dvec3[robot.num_cubes - N];
    int count = 0;
    dvec3 cube;
    unsigned int n = robot.num_cubes; 
    for (int i = 0; i < n; i++)
    {
        cube = robot.existCubes[i];
        bool exist = false;
        for (int j = 0; j < N; j++)
        {
            if (approximatelyEqual(cube.x, positions[j].x) && approximatelyEqual(cube.y, positions[j].y) && approximatelyEqual(cube.z, positions[j].z))
            {
                exist = true;
            }
        }
        if (exist == false)
        {
            newExist[count] = cube;
            ++ count;
        }
    }
    robot = create_robot(newExist, robot.num_cubes - N);
    delete[] newExist;
}

// delete a cube, this is for mutation
__device__ knl_Robot deleteCube(knl_Robot &robot, double x, double y, double z)
{
    int index = 0;
    dvec3 cube;
    int n = robot.num_cubes;
    for (int i = 0; i < n; i++)
    {
        cube = robot.existCubes[i];
        if (approximatelyEqual(cube.x, x) && approximatelyEqual(cube.y, y) && approximatelyEqual(cube.z, z))  break;
        ++ index;
    }
    dvec3* newExist;
    newExist = new dvec3[n - 1];
    for (int i = 0; i < index; i++) {
        newExist[i] = robot.existCubes[i];
    }
    for (int i = index + 1; i < n - 1; i++ ) {
        newExist[i - 1] = robot.existCubes[i];
    }
    robot = create_robot(newExist, n - 1);
    return robot;
}


// simulating the robot, return the distance of the robot moveing 
__global__ void glb_simulate(knl_Robot* robots, const unsigned int N, double* distances) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= N) return;
    
    double totalDuration = SIMULATE_DURATION;
    double runningTime = 0;
    double step = SIMULATE_DELTA; 

    // reset mass to keep every robot has the same mass
    for (unsigned int j = 0; j < robots[i].num_masses; ++ j) {
        robots[i].masses[j].m = OVERALL_MASS / robots[i].num_masses;
    }

    dvec2 startPos = knl_getRobotPosition(robots[i]);

    // this is the motor of the robot
    while (runningTime < totalDuration) {
        for (unsigned int j = 0; j < 28; ++ j) {
           robots[i].springs[j].L0 = robots[i].springs[j].a + 0.02 * sin(OMEGA * runningTime + 0.01 * M_PI);
        }
        knl_updateRobot(robots[i]);
        runningTime += step;
    }
    dvec2 endPos = knl_getRobotPosition(robots[i]);
    distances[i] = knl_distance(startPos, endPos);
}

// setup random kernel
 __global__ void setup_kernel(curandState *states) {
         int id = threadIdx.x + blockDim.x * blockIdx.x;
         int seed= id; // different seed for each thread;
         curand_init(seed, id, 0, &states[id]); // Initialize CURAND
 }

__global__ void glb_selection(double* distances, int* winners, curandState* d_state) {
    int tid = threadIdx.x;
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    __shared__ double dist[10];
    __shared__ double local_distance[10];
    // load data to shared memory
    if (tid < 10) dist[tid] = distances[idx];
    __syncthreads();
    for (int i = 0; i <= tid; i++) {
        local_distance[tid] += dist[i];
    }
    __syncthreads();
    // tournament selection, select 6 winners in each group
    // the selection probability is propotional to it's fitness, which is the moving distance
    if (tid < 6) {
        double randomChoice = local_distance[9] * generate(d_state, idx);
            if (randomChoice < local_distance[0]) {
            winners[blockIdx.x * 6 + tid] = blockIdx.x * blockDim.x;
            }
        else {
            for (int i = 0; i < 9; i++) {
                if (randomChoice > local_distance[i] && randomChoice < local_distance[i + 1])
                    {
                        winners[blockIdx.x * 6 + tid] = blockIdx.x * blockDim.x + i + 1;
                    }
            }
        }
    }
}

//device mutate function, add or delete a cube to the robot
__global__ void glb_mutate (knl_Robot* robots, const unsigned N, curandState *d_state) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= N) return;
    double mutation = generate(d_state, idx);
    if (mutation > MUTATE_PROB) return;
    robots[idx] = create_robot(robots[idx].existCubes, robots[idx].num_cubes);
    int mutateType = (int)2 * generate(d_state, idx);
    if (robots[idx].num_cubes > 40) {
        mutateType = 0;
    } else if (robots[idx].num_cubes < 15) {
        mutateType = 1;
    } else {
        mutateType = mutateType;
    }
    // mutate to add a cube
    if (mutateType == 0) {
        int mutate1 = 0;
        int times = 0;
        while (mutate1 < 1) {
            // generate a random number between 0, cube_num - 1
            int choice = (int)robots[idx].num_cubes * generate(d_state, idx);
            if (canDelete(robots[idx], robots[idx].existCubes[choice]) == true) {
                dvec3 delete_position = {robots[idx].existCubes[choice].x, robots[idx].existCubes[choice].y, robots[idx].existCubes[choice].z};
                dvec3* deletep = new dvec3[1];
                deletep[0] = delete_position;
                deleteCubes(robots[idx], deletep, 1);
                mutate1++;
                delete[] deletep;
            }
            else {
                choice = (int)(1000 * generate(d_state, idx)) % robots[idx].num_cubes;
                times++;
                if (times > 4) {
                    mutateType = 1;
                    mutate1 = 1;
                }
            }
        }
    } else {
        int mutate2 = 0;
        while (mutate2 < 1) {
            // choose a cube randomly
            int randomChoiceCube, randomChoiceFace;
            randomChoiceCube = (int)robots[idx].num_cubes * generate(d_state, idx);
            dvec3 cube = robots[idx].existCubes[randomChoiceCube];
            // choose a face randomly
            randomChoiceFace = (int)6 * generate(d_state, idx);
            switch (randomChoiceFace) {
                case 0:
                    if (robots[idx].check_exist(cube.x + 0.1, cube.y, cube.z))
                        break;
                    else {
                        robots[idx].create_cube(cube.x + 0.1, cube.y, cube.z);
                        mutate2++;
                        break;
                    }

                case 1:
                    if (robots[idx].check_exist(cube.x - 0.1, cube.y, cube.z))
                        break;
                    else {
                        robots[idx].create_cube(cube.x - 0.1, cube.y, cube.z);
                        mutate2++;
                        break;
                    }

                case 2:
                    if (robots[idx].check_exist(cube.x, cube.y + 0.1, cube.z))
                        break;
                    else {
                        robots[idx].create_cube(cube.x, cube.y + 0.1, cube.z);
                        mutate2++;
                        break;
                    }

                case 3:
                    if (robots[idx].check_exist(cube.x, cube.y - 0.1, cube.z))
                        break;
                    else {
                        robots[idx].create_cube(cube.x, cube.y - 0.1, cube.z);
                        mutate2++;
                        break;
                    }

                case 4:
                    if (robots[idx].check_exist(cube.x, cube.y, cube.z + 0.1))
                        break;
                    else {
                        robots[idx].create_cube(cube.x, cube.y, cube.z + 0.1);
                        mutate2++;
                        break;
                    }
                case 5:
                    if (robots[idx].check_exist(cube.x, cube.y, cube.z - 0.1) || cube.z - 0.05 < 0)
                        break;
                    else {
                        robots[idx].create_cube(cube.x, cube.y, cube.z - 0.1);
                        mutate2++;
                        break;
                    }
                default:
                    break;
            }
        }
    }
}
// crossover kernel, change a layer of the parent robots
__global__ void glb_crossover(knl_Robot* robots, const unsigned N, curandState *d_state) {
    int idx = blockDim.x * blockIdx.x + threadIdx.x;
    if (idx >= N) return;
    int choice1 = (int)N * generate(d_state, idx);
    int choice2 = (int)N * generate(d_state, idx);
    knl_Robot parent1 = robots[choice1];
    knl_Robot parent2 = robots[choice2];
    knl_Robot child;

    double height1 = 0.0;
    double height2 = 0.0;

    for (int i = 0; i < parent1.num_cubes; ++i) {
        if (parent1.existCubes[i].z > height1) height1 = parent1.existCubes[i].z;
    }
    for (int i = 0; i < parent2.num_cubes; ++i) {
        if (parent2.existCubes[i].z > height2) height2 = parent2.existCubes[i].z;
    }

    // chose a layer they both have
    double height;
    if (height1 > height2) height = height2;
    else height = height1;

    int randomChoice = ((int)((height + 0.1) / 0.1) - 1) * generate(d_state,idx);
    height = 0.1 * (double) (randomChoice);

    dvec3 res[100];
    int count1 = 0;
    int count2 = 0;
    for (int i = 0; i < parent1.num_cubes; i++) {
        if (approximatelyEqual(parent1.existCubes[i].z, height)) {
            res[count1] = parent1.existCubes[i];
            ++ count1;
        }
    }
    for (int i = 0; i < parent2.num_cubes; i++) {
        if  (!approximatelyEqual(parent2.existCubes[i].z, height)) {
            res[count2 +count1] = parent2.existCubes[i];
            ++ count2;
        }
    }
    __syncthreads();
   for (int i = 0; i < 60; i++) {
    robots[idx].existCubes[i] = res[i];

    }
    robots[idx].num_cubes = count1 + count2;
    __syncthreads();
}
