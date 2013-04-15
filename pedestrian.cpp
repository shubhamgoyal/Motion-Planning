#include "pedestrian.h"

Pedestrian::Pedestrian(State astate, Pedestrian_Behavior behavior, long long int timeSteps):Object(astate) {
	this->behavior = behavior;
	color = 0;
}

void Pedestrian::update_state(double time_step) {
	behavior.update_state(actions, time_step, this->state);
}

void Pedestrian::draw()
{
	if (color)
	{
		glColor3f(1.0,0,0);
	}
	else
	{
		glColor3f(1.0,1.0,1.0);
	}
	glPushMatrix();
   glTranslatef(state.x,state.y,0);
   glRotatef(state.theta*180/M_PI,0,0,1);
	glPointSize(PEDESTRIAN_SIZE);
   glBegin(GL_POINTS);
	glVertex2f(0,0);	
   glEnd();
   glPopMatrix();

}
