#ifndef CONFIG_H
#define CONFIG_H

static const double MASS = 0.1;
static const double LENGTH = 0.1;
static const double LNEGTH = 0.1;
static const double GRAVITY = 9.81;
static const double DAMPING = 1;
static const double DT = 0.001;
static const double OVERALL_MASS = 10;

static const double OMEGA = 50;
static const double K_GROUND = 5000;
static const double MU = 0.8;

static const int SPRING_CONSTANT = 5000;

static const unsigned int DIM = 3;

static const double SIMULATE_DURATION = 1;  // 5000 ms, 5 seconds
static const double SIMULATE_DELTA = 0.001;  // 1ms per step

static const bool EVOLUTION = true;

static const double SELECT_PRESSURE = 0.4;
static const double MUTATE_PROB = 0.2;


#endif
