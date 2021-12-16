#include "kernel.cuh"
#include "stdio.h"
#include "config.hpp"
#include "math_constants.h"
#include "device_launch_parameters.h"

inline __device__ dvec2 knl_getRobotPosition(const knl_Robot& robot) {
    unsigned int N = robot.num_masses;
    double x = 0, y = 0;
    for (unsigned int i = 0; i < N; ++ i) {
        x += robot.masses[i].p.x;
        y += robot.masses[i].p.y;
    }
    return { x / N, y / N };
}

inline __device__ double knl_distance(const dvec3& p1, const dvec3& p2) {
    const double dx = p1.x - p2.x, dy = p1.y - p2.y, dz = p1.z - p2.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

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


inline __device__ dvec3 knl_normalize(const dvec3& p1) {
    const double magnitude = sqrt(p1.x * p1.x + p1.y * p1.y + p1.z * p1.z);
    if (magnitude < 1e-6) {
        return { 0, 0, 0 };
    }
    return { p1.x / magnitude, p1.y / magnitude, p1.z / magnitude };
}

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
                // printf("Updating 1, robot.masses[spring.m2].p=%f, robot.masses[spring.m1].p=%f\n", robot.masses[spring.m2].p.y, robot.masses[spring.m1].p.y);
            } else if (spring.m2 == i) {
                auto springLength = knl_distance(robot.masses[spring.m1].p, robot.masses[spring.m2].p);
                auto springDirection = knl_normalize(knl_subtract(robot.masses[spring.m1].p, robot.masses[spring.m2].p));
                auto springForce = knl_multiply(springDirection, spring.k * (springLength - spring.L0));
                knl_add_to(mass.force, springForce);
                // printf("Updating 1, robot.masses[spring.m2].p=%f, robot.masses[spring.m1].p=%f\n", robot.masses[spring.m2].p.y, robot.masses[spring.m1].p.y);
            }
        }
        // printf("Updating, mass.force.x=%f\n", mass.force.x);
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

        // printf("Updating, springDirection.x=%f\n", springDirection.x);
        // printf("Updating, springLength=%f, spring.L0=%f, spring.k=%f\n", springLength, spring.L0, spring.k);
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

__global__ void glb_simulate(knl_Robot* robots, const unsigned int N, double* distances) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    // printf("i = %f\n", i);
    if (i >= N) return;
    
    double totalDuration = SIMULATE_DURATION;
    double runningTime = 0;
    double step = SIMULATE_DELTA;

    // printf("num mass: %d, num springs: %d\n", robots[i].num_masses, robots[i].num_springs);

    // reset mass
    for (unsigned int j = 0; j < robots[i].num_masses; ++ j) {
        robots[i].masses[j].m = OVERALL_MASS / robots[i].num_masses;
    }

    dvec2 startPos = knl_getRobotPosition(robots[i]);

    while (runningTime < totalDuration) {
        // Why 28?
        for (unsigned int j = 0; j < 28; ++ j) {
            // printf("j=%d, num\n", j);
            robots[i].springs[j].L0 += robots[i].springs[j].a + 0.02 * sin(OMEGA * runningTime + 0.01 * M_PI);
        }
        // printf("One round of updated\n");
        knl_updateRobot(robots[i]);
        runningTime += step;
    }
    dvec2 endPos = knl_getRobotPosition(robots[i]);
    distances[i] = knl_distance(startPos, endPos);
    // printf("start: %f, %f, end: %f, %f\n", startPos.x, startPos.y, endPos.x, endPos.y);
}
