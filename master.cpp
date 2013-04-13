#include "master.h"
#include <cstdio>

void initialize_environment() {
	printf("-----INITIALIZE-----\n");
	for (int i = 0; i < NUMBER_OF_PEDESTRIANS; i++) {
		printf("%d\n",i);
		pedestrians.push_back( *(new Pedestrian( *(new Pedestrian_Behavior()), NUMBER_OF_TIMESTEPS)));
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
