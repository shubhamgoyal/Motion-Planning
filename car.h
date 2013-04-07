#ifndef CAR
#define CAR
#include <deque>
#include <GL/glut.h>
#define _USE_MATH_DEFINES
#include <cmath>

#include "Object.h"
#define CARLENGTH 3
#define CARWIDTH 1.6


class Car: public Object {
	public:
		Car():Object(){}

		Car(dd l, dd w):Object(){
			carLength = l;
			carWidth = w;
		};

		Car(State astate, dd l, dd w):Object(astate){
			carLength = l;
			carWidth = w;
		};

		Car(State astate):Object(astate){};

		Car(dd ax, dd ay, dd av, dd atheta, dd l, dd w):Object(ax,ay,av,atheta){
			carLength = l;
			carWidth = w;
		};

		void update(dd time_step){
			state.x += getXDot()*time_step;
			state.y += getYDot()*time_step;
		}

		void control(){
			Control curControl;
			if (!path.empty()) {
				curControl = path.front();
				path.pop_front();
			}
			else {
				curControl.h1 = 0;
				curControl.h2 = 0;
			}

			state.v += curControl.h1;
			state.theta += state.v*tan(curControl.h2)/carLength;
			if (state.v < 0) state.v = 0;
		}

		void control(dd h1, dd h2){
			state.v += h1;
			state.theta += state.v*tan(h2)/carLength;
		}

		dd getLength(){
			return carLength;
		}

		dd getWidth(){
			return carWidth;
		}

		void setPath(std::deque <Control> apath)
		{
			path = apath;
		}

		void draw()
		{
			glPushMatrix();
			glTranslatef(state.x,state.y,0);
			glRotatef(state.theta*180/M_PI,0,0,1);
			glBegin(GL_QUADS);
			glVertex2f(-carWidth/2,-carLength/2);
			glVertex2f(carWidth/2,-carLength/2);
			glVertex2f(carWidth/2,carLength/2);
			glVertex2f(-carWidth/2,carLength/2);
			glEnd();
			glPopMatrix();
		}

	private:
		dd carLength = CARLENGTH;
		dd carWidth = CARWIDTH;
		std::deque <Control> path;
};

#endif
