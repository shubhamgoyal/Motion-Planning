#ifndef STRUCTS
#define STRUCTS

//TYPEDEF
typedef double dd;

//DEFINITION
#define _USE_MATH_DEFINES
#include<cmath>

//for car
#define CARLENGTH 3
#define CARWIDTH 1.6

//for pedestrian
#define PEDESTRIAN_SIZE 5

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
