#ifndef MASTER
#define MASTER

#include <vector>

#include "pedestrian.h"
#include "car.h"
#include "environment.h"
#include "SimplePlanner.h"
#include <pthread.h>

std::vector<Pedestrian> pedestrians;
Car car;
SimplePlanner planner;

#endif
