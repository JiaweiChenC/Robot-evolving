#ifndef KERNEL_H
#define KERNEL_H

#include "cuda.h"
#include "cuda_runtime.h"
#include "cuda_runtime_api.h"

#include <curand_kernel.h>
#include <curand.h>
// kernel optimized structs
struct dvec3 {
    double x = 0, y = 0, z = 0;
};

struct dvec2 {
    double x = 0, y = 0;
};

struct knl_Mass {
    double m;
    dvec3 p, v, a, force;
};

struct knl_Spring {
    double L0, a, k;
    int m1, m2;
};


struct knl_Robot {
    // a robot has less than 2000 springs and less than 500 masses
    knl_Spring* springs;
    knl_Mass*  masses;
    knl_Mass knl_masses[500];
    knl_Spring knl_springs[1500];
    dvec3 existCubes[100] = {-10, 0, 0};
    unsigned int num_springs = 0;
    unsigned int num_masses = 0;
    unsigned int num_cubes = 0;
    __device__  void create_cube(double x, double y, double z);
    __device__  bool check_exist(double x, double y, double z);

};

__global__ void glb_selection(double*, int*, curandState*);
__global__ void glb_crossover(knl_Robot*, const unsigned int, curandState*);
__global__ void glb_simulate(knl_Robot*, const unsigned int, double*);
__global__ void glb_mutate(knl_Robot*, const unsigned int, curandState*);
__global__ void setup_kernel(curandState*);

#endif
