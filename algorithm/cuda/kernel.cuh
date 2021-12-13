#ifndef KERNEL_H
#define KERNEL_H

#include "cuda.h"
#include "cuda_runtime.h"
#include "cuda_runtime_api.h"

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
    knl_Spring* springs;
    unsigned int num_springs;
    knl_Mass* masses;
    unsigned int num_masses;
};

__global__ void glb_simulate(knl_Robot*, const unsigned int, double*);

#endif
