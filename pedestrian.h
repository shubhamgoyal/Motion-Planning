#ifndef PEDESTRIAN
#define PEDESTRIAN

#include <vector>
#include <queue>
#include <GL/glut.h>
#include "Object.h"
#include "pedestrian_actions.h"
#include "pedestrian_behavior.h"

class Pedestrian: public Object {

protected:
	//std::vector<pedestrian::action> actions;
	std::queue<pedestrian::action> actions;
	Pedestrian_Behavior behavior;
	int color;
	int action_type;

public:
	Pedestrian(State astate, Pedestrian_Behavior behavior, long long int timeSteps);
	void update_state(double time_step);
	void draw();
	int getColor()
	{
		return color;
	}
	void setColor(int c)
	{
		color=c;
	}
};

#endif
