#ifndef STRUCTS
#define STRUCTS

//TYPEDEF
typedef double dd;

//DEFINITION
#define _USE_MATH_DEFINES
#include<cmath>
#define RANDOM_SEED 12345

//for car
#define CARLENGTH 3.0
#define CARWIDTH 1.6
#define Y_VISIBLE 100

//for pedestrian
#define PEDESTRIAN_SIZE 5

//for planner
#define MAX_V 25.0
#define MAX_DECEL 0.7e-1

//STRUCT
struct State {
	dd x;
	dd y;
	dd v;
	dd theta;
} ;

struct Control {
	dd h1;
	dd h2;
} ;

#endif
