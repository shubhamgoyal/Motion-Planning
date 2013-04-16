/*

The range of speed of pedestrian is [1, 2] m/s.

The initial position of every pedestrian is on the pavement.

Every pedestrian has a goal. A goal can be either of the following:-
1. Walk toward the grass parallel to the x-axis (this is the exit action)
2. Go to a particular point on the opposite pavement to meet someone
3. Go to a particular point (different from current location) on this pavement.
4. Just stop and laze around

So, I initially randomly pick a value from {1, 2, 3, 4}. Each value corresponds to the corresponding
action from the list above.

Case 1: First, the speed of pedestrian for the journey is sampled.
		While getting out, at each time step, the pedestrian has a small probability to stop
		for a while. The number of time stpes for which the pedestrian 
		stops < 3s/time_for_each_timestep. I sample numbers from 1 to 10 and if I get 1,
		then the pedestrian stops. I again sample the number of timesteps for which the pedestrian
		wants to stop. The pedestrian stops for that many time steps and then continues on its
		journey.

Case 2: The first step is to sample the goal. The goal is a location on the other pavement.
		
		We assume that the pedestrian takes a U-shaped path towards the goal. So, we assume
		that the pedestrian first walks to the y co-ordinate at which it wants to cross the
		road. Then, it crosses the road. Then, it walks to the target.
		
		The second step is to sample the crossing y co-ordinate. This is done using a weighted
		probability distribution where the probability of a point being picked up is in the following order:

		zebra_crossing_within_range (if it exists) >
		zebra_crossing_outside_range (the probability of different zebra crossings outside
		range varies and is inversely proportional to distance)>
		non_zebra_crossing_point_within_range >
		non_zebra_crossing_point_outside_range (the probabilities inversely vary with distance)

		Once this point is determined, the rest of the path is deterministic.

		The speed of pedestrian is sampled 3 times. First, it is sampled for the initial
		leg of the journey (from current location to a point on the same pavement). Then, it
		is sampled again for the second leg (roac crossing journey). It is sampled the third time
		for the rest of the journey.

		Just like in Case 1, there is a small probability that the pedestrian stops in between its journey.

Case 3: We assume that x co-ordinate of the goal location is the same as the pedestrian's
		current location.

		We sample y co-ordinate.

		The pedestrian walks towards the y co-ordinate. The speed is sampled.

		Just like in Case 1, there is a small probability that the pedestrian stops in between its journey.

Case 4: We sample a random number and the pedestrian stops for that many time steps (the number of timesteps
		has to be within [3s/time_for_each_timestep, 15s/time_for_each_timestep]).
*/

#ifndef PEDESTRIAN_BEHAVIOR
#define PEDESTRIAN_BEHAVIOR

#include "environment.h"
#include "typeAndStruct.h"
#include "pedestrian_actions.h"

#include <queue>
#include <time.h>
#include <stdlib.h>

#define MIN_PEDESTRIAN_SPEED 0.5
#define MAX_PEDESTRIAN_SPEED 2.0

class Pedestrian_Behavior {
protected:
	void insert_new_long_term_goal (std::queue<pedestrian::action> &actions, State& pedestrian_state, int& action_type);
	void insert_long_term_exit(std::queue<pedestrian::action> &actions, State& pedestrian_state);
	void insert_long_term_walk_same_pavement(std::queue<pedestrian::action> &actions, State& pedestrian_state);
	void insert_long_term_walk_opposite_pavement(std::queue<pedestrian::action> &actions, State& pedestrian_state);
	void insert_long_term_stop(std::queue<pedestrian::action> &actions);
	double sample_random(double min_value, double max_value);
	double sample_normal_random(double min_value, double max_value, double mean, double stddev);
	int isExit;
public:
	Environment environment;
	Pedestrian_Behavior()
	{
		isExit = 0;
		environment = Environment();

	};
	void update_state (std::queue<pedestrian::action> &actions, double time_step, State& pedestrian_state, int& action_type);


private:
	static int getRand()
	{
		static int isSeeded=0;
		if (isSeeded == 0){
			srand( (unsigned int)RANDOM_SEED*RANDOM_SEED*343+38 );
			isSeeded=1;
		}
		return rand();
	}


};
	

#endif
