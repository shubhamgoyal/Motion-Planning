#include "pedestrian_behavior.h"
#include <stdio.h>
#include <cassert>
#include <cmath>
#include <random>
#include <vector>

void Pedestrian_Behavior::update_state (std::queue<pedestrian::action> &actions, double time_step, State& pedestrian_state) {
	if (actions.empty()) {
		insert_new_long_term_goal(actions, pedestrian_state);
	}
	pedestrian::action next_action = actions.front();
	/*if (next_action.is_short_term_stop) {
		actions.pop();
	}
	else {*/
	/*DEBUG*/
	//printf("Size of actions: %d\n", actions.size());
		
	/**/
	pedestrian_state.x = pedestrian_state.x + next_action.x_velocity * time_step;
	pedestrian_state.y = pedestrian_state.y + next_action.y_velocity * time_step;
	assert(!actions.empty());
	actions.pop();
}

void Pedestrian_Behavior::insert_new_long_term_goal(std::queue<pedestrian::action> &actions, State& pedestrian_state) {
	printf("In insert goal\n");
	int goal_type = (getRand() % 4) + 1;
	/*DEBUG*/
	//goal_type = 1;
	printf("Goal type is %d\n", goal_type);
	/**/
	switch (goal_type) {
		case 1:
			insert_long_term_exit(actions, pedestrian_state);
			break;
		case 2:
			insert_long_term_walk_same_pavement(actions, pedestrian_state);
			break;
		case 3:
			insert_long_term_walk_opposite_pavement(actions, pedestrian_state);
			break;
		case 4:
			insert_long_term_stop(actions);
			break;
		default:
			printf("Wrong value sampled\n");
	}
}

void Pedestrian_Behavior::insert_long_term_exit(std::queue<pedestrian::action> &actions, State& pedestrian_state) {
	printf("In long_term_exit\n");
	double target_x;
	double random_velocity = 0;
	while (random_velocity == 0.0) {
		if (pedestrian_state.x <= PAVEMENT_LEFT_X_MAX) {
			target_x = X_MIN;
			random_velocity = 0.0 - sample_random(MIN_PEDESTRIAN_SPEED, MAX_PEDESTRIAN_SPEED);
		}
		else {
			target_x = X_MAX;
			random_velocity = sample_random(MIN_PEDESTRIAN_SPEED, MAX_PEDESTRIAN_SPEED);
		}
	}
	int num_time_steps_needed = (int)ceil(((target_x - pedestrian_state.x) / random_velocity)/TIME_STEP_DURATION);
	/*DEBUG*/
	printf("num_time_steps_needed: %d\n",num_time_steps_needed);
	printf("target_x: %lf, pedestrian_state.x: %lf\nrandom_velocity: %lf\n",target_x, pedestrian_state.x, random_velocity);
	/**/
	
	for (int i = 0; i < num_time_steps_needed; i++) {
		pedestrian::action new_action = {random_velocity, 0.0};
		actions.push(new_action);
	}
	for (int i=0;i <NUMBER_OF_TIMESTEPS;++i)
	{
		pedestrian::action new_action = {0.0, 0.0};
		actions.push(new_action);
	}
}

double Pedestrian_Behavior::sample_random(double min_value, double max_value) {
	std::uniform_real_distribution<double> dist(min_value, max_value);
	std::mt19937 rng;
	rng.seed(std::random_device{}()); 
	return dist(rng);
}

double Pedestrian_Behavior::sample_normal_random(double min_value, double max_value, double mean, double stddev) {
	std::uniform_real_distribution<double> dist(mean, stddev);
	std::mt19937 rng;
	rng.seed(std::random_device{}()); 
	double value = 100000;
	while ((value > Y_MAX) || (value < Y_MIN)) {
		value = dist(rng);
	}
	return value;
}

void Pedestrian_Behavior::insert_long_term_walk_same_pavement(std::queue<pedestrian::action> &actions, State& pedestrian_state) {
	printf("In long_term_walk_same_pavement\n");
	double sample_goal_location_y = pedestrian_state.y;
	while (sample_goal_location_y == pedestrian_state.y) {
		sample_goal_location_y = sample_random(Y_MIN, Y_MAX);
	}
	double random_velocity = 0.0;
	while (random_velocity == 0.0) {
		random_velocity = sample_random(MIN_PEDESTRIAN_SPEED, MAX_PEDESTRIAN_SPEED);
	}
	if (sample_goal_location_y < pedestrian_state.y) random_velocity *= -1.0;
	int num_time_steps_needed = (int)ceil(((sample_goal_location_y - pedestrian_state.y) / random_velocity)/TIME_STEP_DURATION);
	num_time_steps_needed = abs(num_time_steps_needed);
	assert((printf("\ngoal:(%lf,%lf), cur:(%lf,%lf),v:%lf, numStep:%d\n",pedestrian_state.x, sample_goal_location_y, pedestrian_state.x, pedestrian_state.y, random_velocity, num_time_steps_needed ),num_time_steps_needed > 0));
	for (int i = 0; i < num_time_steps_needed; i++) {
		pedestrian::action new_action = {0.0, random_velocity};
		actions.push(new_action);
	}
}

void Pedestrian_Behavior::insert_long_term_walk_opposite_pavement(std::queue<pedestrian::action> &actions, State& pedestrian_state) {
	printf("In long_term_walk_opposite_pavement\n");
	double sample_goal_location_x;
	double sample_goal_location_y;
	double y_cross;
	if (pedestrian_state.x <= PAVEMENT_LEFT_X_MAX) {
		sample_goal_location_x = sample_random(PAVEMENT_RIGHT_X_MIN, PAVEMENT_RIGHT_X_MAX);
		sample_goal_location_y = sample_random(Y_MIN, Y_MAX);
	}
	else {
		sample_goal_location_x = sample_random(PAVEMENT_LEFT_X_MIN, PAVEMENT_LEFT_X_MAX);
		sample_goal_location_y = sample_random(Y_MIN, Y_MAX);
	}
	std::vector<int> zebra_crossings_within_range;
	for (int i = 0; i < NUM_ZEBRA_CROSSING; i++) {
		if ((((sample_goal_location_y - environment.zebra_crossings[i].y_min) <= 0) && ((pedestrian_state.y - environment.zebra_crossings[i].y_min) >= 0)) ||
			(((sample_goal_location_y - environment.zebra_crossings[i].y_min) >= 0) && ((pedestrian_state.y - environment.zebra_crossings[i].y_min) <= 0)) ||
			(((sample_goal_location_y - environment.zebra_crossings[i].y_max) <= 0) && ((pedestrian_state.y - environment.zebra_crossings[i].y_max) >= 0)) ||
			(((sample_goal_location_y - environment.zebra_crossings[i].y_max) >= 0) && ((pedestrian_state.y - environment.zebra_crossings[i].y_max) <= 0))) {
			zebra_crossings_within_range.push_back(i);
		}
	}
	int random_use_zebra_crossing = (rand() % 10) + 1;
	if  (zebra_crossings_within_range.size() > 0) {
		if (random_use_zebra_crossing > 3) {
			if (zebra_crossings_within_range.size() > 1) {
				//2 zebra crossings within range
				double min_value = 0;
				double max_value = 4;
				double value = sample_random(min_value, max_value);
				if (value <= 2) {
					y_cross = environment.zebra_crossings[zebra_crossings_within_range[0]].y_min + value;
				}
				else {
					y_cross = environment.zebra_crossings[zebra_crossings_within_range[1]].y_min + value - 2;
				}
			}
			else {
				double min_value = 0;
				double max_value = 2;
				double value = sample_random(min_value, max_value);
				y_cross = environment.zebra_crossings[zebra_crossings_within_range[0]].y_min + value;
			}
		}
		else {
			double mid_point_range;
			double min_y = std::min(pedestrian_state.y, sample_goal_location_y);
			double max_y = std::max(pedestrian_state.y, sample_goal_location_y);
			double mean = min_y + abs(max_y - min_y)/2.0;
			double stddev = abs(max_y - min_y)/2.0;
			y_cross = sample_normal_random(Y_MIN, Y_MAX, mean, stddev);
		}
	}
	else {
		if (random_use_zebra_crossing > 7) {
			double min_value = 0;
			double max_value = 4;
			double value = sample_random(min_value, max_value);
			if (value <= 2) {
				y_cross = environment.zebra_crossings[0].y_min + value;
			}
			else {
				y_cross = environment.zebra_crossings[1].y_min + value - 2;
			}
		}
		else {
			double mid_point_range;
			double min_y = std::min(pedestrian_state.y, sample_goal_location_y);
			double max_y = std::max(pedestrian_state.y, sample_goal_location_y);
			double mean = min_y + abs(max_y - min_y)/2.0;
			double stddev = abs(max_y - min_y)/2.0;
			y_cross = sample_normal_random(Y_MIN, Y_MAX, mean, stddev);
		}
	}
	//------------------------Found the cross over point--------------------------------------
	double random_velocity = 0.0;
	while (random_velocity == 0.0) {
		random_velocity = sample_random(MIN_PEDESTRIAN_SPEED, MAX_PEDESTRIAN_SPEED);
	}
	if (y_cross < pedestrian_state.y) random_velocity *= -1.0;
	int num_time_steps_needed = (int)ceil((abs(y_cross - pedestrian_state.y) / random_velocity)/TIME_STEP_DURATION);
	num_time_steps_needed = abs(num_time_steps_needed);
	
	for (int i = 0; i < num_time_steps_needed; i++) {
		pedestrian::action new_action = {0.0, random_velocity};
		actions.push(new_action);
	}

	double random_velocity1 = 0.0;
	while (random_velocity1 == 0.0) {
		random_velocity1 = sample_random(MIN_PEDESTRIAN_SPEED, MAX_PEDESTRIAN_SPEED);
	}
	if (sample_goal_location_x < pedestrian_state.x) random_velocity1 *= -1.0;
	int num_time_steps_needed1 = (int)ceil((abs(sample_goal_location_x - pedestrian_state.x) / random_velocity1)/TIME_STEP_DURATION);
	num_time_steps_needed1 = abs(num_time_steps_needed1);
	for (int i = 0; i < num_time_steps_needed1; i++) {
		pedestrian::action new_action = {random_velocity1, 0.0};
		actions.push(new_action);
	}

	double random_velocity2 = 0.0;
	while (random_velocity2 == 0.0) {
		random_velocity2 = sample_random(MIN_PEDESTRIAN_SPEED, MAX_PEDESTRIAN_SPEED);
	}
	if (sample_goal_location_y < y_cross) random_velocity2 *= -1.0;
	int num_time_steps_needed2 = (int)ceil((abs(sample_goal_location_y - pedestrian_state.y) / random_velocity2)/TIME_STEP_DURATION);
	num_time_steps_needed2 = abs(num_time_steps_needed2);
	for (int i = 0; i < num_time_steps_needed2; i++) {
		pedestrian::action new_action = {0.0, random_velocity2};
		actions.push(new_action);
	}
	assert( (printf("\ncurX:%lf, curY:%lf, cross:%lf, dest: (%lf,%lf)\n v1:%lf, v2:%lf,v3:%lf\nnumStep=%d, numStep1=%d, numStep2=%d\n",pedestrian_state.x, pedestrian_state.y, y_cross, sample_goal_location_x, sample_goal_location_y, random_velocity, random_velocity1, random_velocity2, num_time_steps_needed, num_time_steps_needed1, num_time_steps_needed2), (num_time_steps_needed + num_time_steps_needed1 + num_time_steps_needed2) > 0));
}

void Pedestrian_Behavior::insert_long_term_stop(std::queue<pedestrian::action> &actions) {
	printf("In long_term_stop\n");
	int random_steps = (rand() % 101);
	random_steps += 50;
	for (int i = 0; i < random_steps; i++) {
		pedestrian::action new_action = {0.0, 0.0};
		actions.push(new_action);
	}
}
