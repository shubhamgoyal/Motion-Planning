/*
Assume a rectangular region 500m x 15m in dimensions. A road of width 10m runs at the
centre. Surrounding the road are two pavements of width 2m each. Outside the pavements, there
are grassy regions of width 0.5m on each side. There are two zebra crossings on the road,
one at 1/4th the length of the road and the other at 3/4th the length. Let us call the first
crossing A and the second one B.

The origin lies at the bottom left corner of the rectangle. The x co-ordinate increases
from left to right while the y co-ordinate increases from the bottom to top.
*/

#ifndef ENVIRONMENT
#define ENVIRONMENT

#define X_MIN 0
#define X_MAX 15
#define Y_MIN 0
#define Y_MAX 500
#define PAVEMENT_LEFT_X_MIN 0.5
#define PAVEMENT_LEFT_X_MAX 2.5
#define PAVEMENT_RIGHT_X_MIN 12.5
#define PAVEMENT_RIGHT_X_MAX 14.5
#define PAVEMENT_LEFT_Y_MIN 0
#define PAVEMENT_LEFT_Y_MAX 500
#define PAVEMENT_RIGHT_Y_MIN 0
#define PAVEMENT_RIGHT_Y_MAX 500

#define NUM_ZEBRA_CROSSING 2
struct zebra_crossing {
	double y_min;
	double y_max;
	double x_min;
	double x_max;
	double width;
};

#define NUMBER_OF_PEDESTRIANS 100
#define NUMBER_OF_TIMESTEPS 1000000
#define TIME_STEP_DURATION 0.1

class Environment
{
	public:
		Environment(){};
		zebra_crossing zebra_1 = {124, 126, 2.5, 12.5, 2};
		zebra_crossing zebra_2 = {374, 376, 2.5, 12.5, 2};
		zebra_crossing zebra_crossings [2]= {zebra_1, zebra_2};

};


#endif
