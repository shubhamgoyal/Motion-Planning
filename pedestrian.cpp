#include "pedestrian.h"

Pedestrian::Pedestrian(Pedestrian_Behavior behavior, long long int timeSteps) {
	this->behavior = behavior;
}

void Pedestrian::update_state(double time_step) {
	behavior.update_state(actions, time_step, this->state);
}