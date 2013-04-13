#include "master.h"
#include <cstdio>
#include <ctime>
#include <cstdlib>
#include <random>

int getRandomInt()
{
	static int isSeeded = 0;
	if (!isSeeded)
	{
		srand(time(NULL));
		isSeeded = 1;
	}
	return rand();
}

State genRandomPedestrianState()
{
	State st;
	std::uniform_real_distribution<double> dist(0,1);
	std::mt19937 rng;
	rng.seed(std::random_device{}());
	if (getRandomInt()%2 == 0)
	{
		st.x = PAVEMENT_LEFT_X_MIN + dist(rng)*(PAVEMENT_LEFT_X_MAX - PAVEMENT_LEFT_X_MIN);
	}
	else
	{
		st.x = PAVEMENT_RIGHT_X_MIN + dist(rng)*(PAVEMENT_RIGHT_X_MAX - PAVEMENT_RIGHT_X_MIN);
	}
	
	st.y = PAVEMENT_LEFT_Y_MIN + dist(rng)*(PAVEMENT_LEFT_Y_MAX - PAVEMENT_LEFT_Y_MIN);
	st.v = 0;
	st.theta = 0;

}

void initialize_environment() {
	printf("-----INITIALIZE-----\n");
	for (int i = 0; i < NUMBER_OF_PEDESTRIANS; i++) {
		printf("%d\n",i);
		pedestrians.push_back( *(new Pedestrian( getRandomPedestrianState(),*(new Pedestrian_Behavior()), NUMBER_OF_TIMESTEPS)));
	}
}

void execute() {
	printf("-----EXECUTE-----\n");
	for (int i = 0; i < NUMBER_OF_TIMESTEPS; i++) {
		printf("%d\n",i);
		for (int j = 0; j < NUMBER_OF_PEDESTRIANS; j++) {
			pedestrians[j].update_state(TIME_STEP_DURATION);
		}
	}
}

int main() {
	initialize_environment();
	execute();
}
