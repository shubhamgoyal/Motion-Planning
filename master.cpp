#include "master.h"

#define NUMBER_OF_PEDESTRIANS 2
#define NUMBER_OF_TIMESTEPS 100
#define TIME_STEP_DURATION 0.1

void initialize_environment() {
	for (int i = 0; i < NUMBER_OF_PEDESTRIANS; i++) {
		pedestrians.push_back(Pedestrian());
	}
}

void execute() {
	for (int i = 0; i < NUMBER_OF_TIMESTEPS; i++) {
		pedestrians[i].update_state(TIME_STEP_DURATION);
	}
}

int main() {
	initialize_environment();
	execute();
}