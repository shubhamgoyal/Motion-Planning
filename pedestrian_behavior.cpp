#include "pedestrian_behavior.h"
#include <stdio.h>
#include <cassert>
#include <cmath>
#include <random>
#include <vector>

void Pedestrian_Behavior::update_state (std::deque<pedestrian::action> &actions, double time_step, State& pedestrian_state, int& action_type) {	
	State& p = pedestrian_state;
	
	if (isExit && actions.empty()) return;
	if (action_type==0 || actions.empty() ) {
		insert_new_long_term_goal(actions, pedestrian_state, action_type);
	}
	pedestrian::action next_action = actions.front();
	/*if (next_action.is_short_term_stop) {
		actions.pop_front();
	}
	else {*/
	
	double cx = car->getX();
	double cy = car->getY();
	double hlength = car->getLength()/2;
	double hwidth = car->getWidth()/2;
	double vx = fabs(next_action.x_velocity);
	if (vx > 0.01 && p.y < cy+hlength && p.y > cy-hlength && (fabs(cx-p.x)-hwidth)/vx < 1.0 && (cx-p.x)*next_action.x_velocity > 0)
	{
		static int count = 0;
		printf("-----AVOID CAR: %d-----\n", count++);
		//assert (1 != 1);
		pedestrian::action new_action = {0.0, -vx, 500};
		actions.push_front(new_action);
		action_type = action_type%10 + 10;
		next_action = actions.front();
	}
	else if (vx > 0.01)
	{
		action_type %= 10;
	}

	pedestrian_state.x = pedestrian_state.x + next_action.x_velocity * time_step;
	pedestrian_state.y = pedestrian_state.y + next_action.y_velocity * time_step;

	/* To make the pedestrian looping in the environment (if it goes outside??) */
	if (pedestrian_state.y > Y_MAX) pedestrian_state.y -= (Y_MAX-Y_MIN);
	else if (pedestrian_state.y < Y_MIN) pedestrian_state.y += (Y_MAX-Y_MIN);
	/******/
	pedestrian_state.v = sqrt((next_action.x_velocity)*(next_action.x_velocity) +(next_action.y_velocity) *(next_action.y_velocity));
	pedestrian_state.theta = atan2(next_action.y_velocity, next_action.x_velocity);

	assert(!actions.empty());
	if (next_action.time_steps_left >= 1) actions.front().time_steps_left--;
	else 
	{
		if (!actions.empty()) actions.pop_front();
		else action_type=0;
	}
}

void Pedestrian_Behavior::insert_new_long_term_goal(std::deque<pedestrian::action> &actions, State& pedestrian_state, int& action_type) {
	//printf("In insert goal\n");
	int getExit=CHANCE_EXIT;
	int getSamePavement= getExit+CHANCE_SAME_PAVEMENT;
	int getCross=getSamePavement+CHANCE_CROSS;
	int getStop=getCross+CHANCE_STOP;
	int goal_type = (getRand() % getStop) + 1;
	/*DEBUG*/
	//goal_type = 1;
	//printf("Goal type is %d\n", goal_type);
	/**/
	/*
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
	*/

	
	if (goal_type >0 && goal_type <= getExit)
	{
		action_type=-1;
		insert_long_term_exit(actions, pedestrian_state);
	}
	else if (goal_type <= getSamePavement)
	{
		action_type=1;
		insert_long_term_walk_same_pavement(actions, pedestrian_state);
	}
	else if (goal_type <= getCross)
	{
		action_type=2;
		insert_long_term_walk_opposite_pavement(actions, pedestrian_state);
	}
	else if (goal_type <=getStop)
	{
		action_type=3;
		insert_long_term_stop(actions);
	}
	else printf("Wrong value sampled\n");
}

void Pedestrian_Behavior::insert_long_term_exit(std::deque<pedestrian::action> &actions, State& pedestrian_state) {
	//printf("In long_term_exit\n");
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
	//printf("num_time_steps_needed: %d\n",num_time_steps_needed);
	//printf("target_x: %lf, pedestrian_state.x: %lf\nrandom_velocity: %lf\n",target_x, pedestrian_state.x, random_velocity);
	/**/
	/* CHANGED!!
	for (int i = 0; i < num_time_steps_needed; i++) {
		pedestrian::action new_action = {random_velocity, 0.0};
		actions.push_back(new_action);
	}
	*/
	pedestrian::action new_action = {random_velocity, 0.0, num_time_steps_needed};
	actions.push_back(new_action);
	//JUST TO MAKE SURE THAT THE PEDESTRIAN IS STOPPING
	pedestrian::action new_action2 = {0.0,0.0,100};
	actions.push_back(new_action2);

	/*
	for (int i=0;i <NUMBER_OF_TIMESTEPS;++i)
	{
		pedestrian::action new_action = {0.0, 0.0};
		actions.push_back(new_action);
	}
	*/
	isExit=1;
}

double Pedestrian_Behavior::sample_random(double min_value, double max_value) {
	std::uniform_real_distribution<double> dist(min_value, max_value);
	std::mt19937 rng;
	//rng.seed(std::random_device{}()); 
	rng.seed(getRand());
	return dist(rng);
}

double Pedestrian_Behavior::sample_normal_random(double min_value, double max_value, double mean, double stddev) {
	std::normal_distribution<double> dist(mean, stddev);
	std::mt19937 rng;
	//rng.seed(std::random_device{}()); 
	rng.seed(getRand());
	/*
	double value = 100000;
	while ((value > Y_MAX) || (value < Y_MIN)) {
		value = dist(rng);
	}
	*/
	double value = dist(rng);
	if (value > Y_MAX) value = Y_MAX-0.5;
	else if (value < Y_MIN) value = Y_MIN+0.5;

	return value;
}

void Pedestrian_Behavior::insert_long_term_walk_same_pavement(std::deque<pedestrian::action> &actions, State& pedestrian_state) {
	//printf("In long_term_walk_same_pavement\n");
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
	//assert((printf("\ngoal:(%lf,%lf), cur:(%lf,%lf),v:%lf, numStep:%d\n",pedestrian_state.x, sample_goal_location_y, pedestrian_state.x, pedestrian_state.y, random_velocity, num_time_steps_needed ),num_time_steps_needed > 0));
	
	/* CHANGED!!	
	for (int i = 0; i < num_time_steps_needed; i++) {
		pedestrian::action new_action = {0.0, random_velocity};
		actions.push_back(new_action);
	}
	*/
	pedestrian::action new_action = {0.0, random_velocity, num_time_steps_needed};
	actions.push_back(new_action);
}

void Pedestrian_Behavior::insert_long_term_walk_opposite_pavement(std::deque<pedestrian::action> &actions, State& pedestrian_state) {
	//printf("In long_term_walk_opposite_pavement\n");
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

	if (USE_ZEBRA_CROSS)
	{
	std::vector<int> zebra_crossings_within_range;
	for (int i = 0; i < NUM_ZEBRA_CROSSING; i++) {
		if ((((sample_goal_location_y - environment.zebra_crossings[i].y_min) <= 0) && ((pedestrian_state.y - environment.zebra_crossings[i].y_min) >= 0)) ||
			(((sample_goal_location_y - environment.zebra_crossings[i].y_min) >= 0) && ((pedestrian_state.y - environment.zebra_crossings[i].y_min) <= 0)) ||
			(((sample_goal_location_y - environment.zebra_crossings[i].y_max) <= 0) && ((pedestrian_state.y - environment.zebra_crossings[i].y_max) >= 0)) ||
			(((sample_goal_location_y - environment.zebra_crossings[i].y_max) >= 0) && ((pedestrian_state.y - environment.zebra_crossings[i].y_max) <= 0))) {
			zebra_crossings_within_range.push_back(i);
		}
	}
	int random_use_zebra_crossing = (getRand() % 10) + 1;
	if  (zebra_crossings_within_range.size() > 0) {
		if (random_use_zebra_crossing > 3) {
			if (zebra_crossings_within_range.size() > 1) {
				//2 zebra crossings within range
				double min_value = 0.0;
				double max_value = 4.0;
				double value = sample_random(min_value, max_value);
				if (value <= 2.0) {
					y_cross = environment.zebra_crossings[zebra_crossings_within_range[0]].y_min + value;
				}
				else {
					y_cross = environment.zebra_crossings[zebra_crossings_within_range[1]].y_min + value - 2.0;
				}
			}
			else {
				double min_value = 0.0;
				double max_value = 2.0;
				double value = sample_random(min_value, max_value);
				y_cross = environment.zebra_crossings[zebra_crossings_within_range[0]].y_min + value;
			}
		}
		else {
			double mid_point_range;
			double min_y = std::min(pedestrian_state.y, sample_goal_location_y);
			double max_y = std::max(pedestrian_state.y, sample_goal_location_y);
			double mean = min_y + fabs(max_y - min_y)/2.0;
			double stddev = fabs(max_y - min_y)/2.0;
			mean = (pedestrian_state.y + sample_goal_location_y)/2.0;
			y_cross = sample_normal_random(Y_MIN, Y_MAX, mean, stddev);
		}
	}
	else {
		if (random_use_zebra_crossing > 7) {
			double min_value = 0.0;
			double max_value = 4.0;
			double value = sample_random(min_value, max_value);
			if (value <= 2.0) {
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
			double mean = min_y + fabs(max_y - min_y)/2.0;
			double stddev = fabs(max_y - min_y)/2.0;
			mean = (pedestrian_state.y + sample_goal_location_y)/2.0;
			y_cross = sample_normal_random(Y_MIN, Y_MAX, mean, stddev);
		}
	}
	}
	/*To disable the zebra cross
	  Comment to implement the usual
	  */
	else if (USE_ZEBRA_CROSS == 0)
	{
		double mean = (pedestrian_state.y + sample_goal_location_y)/2.0;
		double stddev = fabs(pedestrian_state.y - sample_goal_location_y)/2.0;
		y_cross = sample_normal_random(Y_MIN,Y_MAX,mean,stddev);
	}
	/**/

	//------------------------Found the cross over point--------------------------------------
	double random_velocity = 0.0;
	while (random_velocity == 0.0) {
		random_velocity = sample_random(MIN_PEDESTRIAN_SPEED, MAX_PEDESTRIAN_SPEED);
	}
	if (y_cross < pedestrian_state.y) random_velocity *= -1.0;
	int num_time_steps_needed = (int)ceil(((y_cross - pedestrian_state.y) / random_velocity)/TIME_STEP_DURATION);
	num_time_steps_needed = abs(num_time_steps_needed);
/* CHANGED!!	
	for (int i = 0; i < num_time_steps_needed; i++) {
		pedestrian::action new_action = {0.0, random_velocity};
		actions.push_back(new_action);
	}
*/
	pedestrian::action new_action = {0.0, random_velocity, num_time_steps_needed};
	actions.push_back(new_action);

	assert(sample_goal_location_x <= PAVEMENT_LEFT_X_MAX || sample_goal_location_x >= PAVEMENT_RIGHT_X_MIN);
	double random_velocity1 = 0.0;
	while (random_velocity1 == 0.0) {
		random_velocity1 = sample_random(MIN_PEDESTRIAN_SPEED, MAX_PEDESTRIAN_SPEED);
	}
	if (sample_goal_location_x < pedestrian_state.x) random_velocity1 *= -1.0;
	int num_time_steps_needed1 = (int)ceil(((sample_goal_location_x - pedestrian_state.x) / random_velocity1)/TIME_STEP_DURATION);
	num_time_steps_needed1 = abs(num_time_steps_needed1);
	/* CHANGED!!
	for (int i = 0; i < num_time_steps_needed1; i++) {
		pedestrian::action new_action = {random_velocity1, 0.0};
		actions.push_back(new_action);
	}
	*/
	pedestrian::action new_action1 = {random_velocity1, 0.0, num_time_steps_needed1};
	actions.push_back(new_action1);

	double random_velocity2 = 0.0;
	while (random_velocity2 == 0.0) {
		random_velocity2 = sample_random(MIN_PEDESTRIAN_SPEED, MAX_PEDESTRIAN_SPEED);
	}
	if (sample_goal_location_y < y_cross) random_velocity2 *= -1.0;
	int num_time_steps_needed2 = (int)ceil(((sample_goal_location_y - pedestrian_state.y) / random_velocity2)/TIME_STEP_DURATION);
	num_time_steps_needed2 = abs(num_time_steps_needed2);
	/* CHANGED!! 
	for (int i = 0; i < num_time_steps_needed2; i++) {
		pedestrian::action new_action = {0.0, random_velocity2};
		actions.push_back(new_action);
	}
	*/
	pedestrian::action new_action2 = {0.0, random_velocity2, num_time_steps_needed2};
	actions.push_back(new_action2);

	//assert( (printf("\ncurX:%lf, curY:%lf, cross:%lf, dest: (%lf,%lf)\n v1:%lf, v2:%lf,v3:%lf\nnumStep=%d, numStep1=%d, numStep2=%d\n",pedestrian_state.x, pedestrian_state.y, y_cross, sample_goal_location_x, sample_goal_location_y, random_velocity, random_velocity1, random_velocity2, num_time_steps_needed, num_time_steps_needed1, num_time_steps_needed2), (num_time_steps_needed + num_time_steps_needed1 + num_time_steps_needed2) > 0));
}

void Pedestrian_Behavior::insert_long_term_stop(std::deque<pedestrian::action> &actions) {
	//printf("In long_term_stop\n");
	int random_steps = (getRand() % (int)ceil(50/TIME_STEP_DURATION));
	random_steps += 50;
	/* CHANGED!!
	for (int i = 0; i < random_steps; i++) {
		pedestrian::action new_action = {0.0, 0.0};
		actions.push_back(new_action);
	}
	*/
	pedestrian::action new_action = {0.0, 0.0, random_steps};
	actions.push_back(new_action);
}
