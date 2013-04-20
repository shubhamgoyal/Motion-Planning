#ifndef MASTER
#define MASTER

#include <vector>
#include <ctime>

#include "pedestrian.h"
#include "car.h"
#include "environment.h"
#include "SimplePlanner.h"
#include "PotentialPlanner2.h"
//#include "PotentialPlanner.h"
#include <pthread.h>

std::vector<Pedestrian> pedestrians;
std::vector <Pedestrian*> seenPedestrians;
Car car;
//SimplePlanner planner;
//PotentialPlanner planner;
PotentialPlanner2 planner;

/*For testing */
State previousCarState;
char fname[300]="./DATA/temp";
unsigned int numLightCollision = 0;
unsigned int numHeavyCollision = 0;
unsigned int numCollision[100] = {0};
bool carHitPedestrian(State previousCarState, State currentCarState, State pedestrianState);
clock_t start;
time_t start2;
float timeFromStart=0;
dd yTotal = 0;

#endif
