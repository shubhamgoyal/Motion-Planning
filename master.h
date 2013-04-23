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
char fname[500]="./DATA/";
char fname2[500]="./DATA/";
unsigned int numLightCollision = 0;
unsigned int numHeavyCollision = 0;
unsigned int numCollision[100] = {0};
bool carHitPedestrian(State previousCarState, State currentCarState, State pedestrianState);
time_t start2;
double curTime;
float timeFromStart=0;
dd yTotal = 0;

/* For drawing text */
double ww = 110.0, ww2=95.0, ww3=100.0;
double chWidth[80]={
0.9*	/*Default*/ww,/*A*/ ww,/*B*/ ww,/*C*/ ww,/*D*/ ww,/*E*/ ww,
/*F*/ 0.9*ww,/*G*/ ww,/*H*/ ww,/*I*/ ww/2,/*J*/ 0.7*ww,
/*K*/ ww,/*L*/ 0.9*ww,/*M*/ 1.15*ww ,/*N*/ ww,/*O*/ ww,
/*P*/ ww,/*Q*/ ww,/*R*/ ww,/*S*/ ww,/*T*/ 0.9*ww,
/*U*/ ww,/*V*/ 1.05*ww,/*W*/ 1.2*ww,/*X*/ 0.9*ww,/*Y*/ ww,/*Z*/ ww,
/*a*/ww2, /*b*/ ww2, /*c*/ ww2, /*d*/ ww2, /*e*/ ww2,
/*f*/ 0.65*ww2, /*g*/ ww2, /*h*/ ww2, /*i*/ 0.5*ww2, /*j*/ 0.6*ww2,
/*k*/ ww2, /*l*/ 0.5*ww2, /*m*/ 1.5*ww2, /*n*/ ww2, /*o*/ ww2,
 /*p*/ 1.05*ww2, /*q*/ ww2, /*r*/ 0.8*ww2, /*s*/ ww2, /*t*/ 0.7*ww2,
 /*u*/ ww2, /*v*/ ww2, /*w*/ 1.2*ww2, /*x*/ 0.9*ww2, /*y*/ ww2, /*z*/ 0.9*ww2,
/*0*/ww3, /*1*/ 0.8*ww3, /*2*/ ww3, /*3*/ ww3, /*4*/ ww3,
/*5*/ ww3, /*6*/ ww3, /*7*/ ww3, /*8*/ ww3, /*9*/ ww3};

#endif
