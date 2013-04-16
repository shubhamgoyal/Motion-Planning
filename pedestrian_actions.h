#ifndef PEDESTRIAN_ACTIONS
#define PEDESTRIAN_ACTIONS

/*#define ACTION_EXIT 1
#define ACTION_WALK 2
#define ACTION_CROSS 3
#define STOP 4*/

namespace pedestrian {
	struct action {
		//int to_do;
		//bool is_short_term_stop;
		double x_velocity;
		double y_velocity;
		int time_steps_left;
	};
}

#endif
