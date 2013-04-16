#include "pedestrian.h"

Pedestrian::Pedestrian(State astate, Pedestrian_Behavior &behavior, long long int timeSteps):Object(astate) {
	this->behavior = behavior;
	color = 0;
	action_type=0;
	update_state(TIME_STEP_DURATION);
}

void Pedestrian::update_state(double time_step) {
	behavior.update_state(actions, time_step, this->state, this->action_type);
}

void Pedestrian::draw()
{
	if (color)
	{
		glColor3f(1.0,0,0);
	}
	else
	{
		if (action_type >= 10) 
		{
			glColor3f(0.0,0.8,1.0);
		}
		else
		{
			switch (action_type) {
				case 0:
					glColor3f(1.0,1.0,1.0);
					break;
				case -1:
					glColor3f(1.0,1.0,1.0);
					break;
				case 1:
					glColor3f(0.0,0.0,1.0);
					break;
				case 2:
					glColor3f(0.0,1.0,0.0);
					break;
				case 3:
					glColor3f(0.8, 0.8, 0.8);
					break;
				default:
					glColor3f(1.0,1.0,1.0);
	
			}
		}
	}
	glPushMatrix();
   glTranslatef(state.x,state.y,0);
   glRotatef(state.theta*180/M_PI,0,0,1);
	/*
	glPointSize(PEDESTRIAN_SIZE);
   glBegin(GL_POINTS);
	glVertex2f(0,0);	
   glEnd();
	*/
	float p= PEDESTRIAN_SIZE/20.0;
	glBegin(GL_QUADS);
		glVertex2f(-p/2,-p);
		glVertex2f(-p/2,p);
		glVertex2f(p/2,p);
		glVertex2f(p/2,-p);
	glEnd();
   glPopMatrix();

}
