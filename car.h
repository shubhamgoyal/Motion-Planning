#ifndef CAR
#define CAR
#include <deque>
#include <GL/glut.h>
//#define _USE_MATH_DEFINES
//#include <cmath>

#include "Object.h"
#include <pthread.h>
//#define CARLENGTH 3.0
//#define CARWIDTH 1.6


class Car: public Object {
	public:
		Car():Object(){
			pthread_mutex_init(&mutex_path, NULL);
			carLength = CARLENGTH;
			carWidth = CARWIDTH;
		}

		Car(dd l, dd w):Object(){
			pthread_mutex_init(&mutex_path, NULL);
			carLength = l;
			carWidth = w;
		};

		Car(State astate, dd l, dd w):Object(astate){
			pthread_mutex_init(&mutex_path, NULL);
			carLength = l;
			carWidth = w;
		};

		Car(State astate):Object(astate){
			pthread_mutex_init(&mutex_path, NULL);
			carLength = CARLENGTH;
			carWidth = CARWIDTH;
		};

		Car(dd ax, dd ay, dd av, dd atheta, dd l, dd w):Object(ax,ay,av,atheta){
			pthread_mutex_init(&mutex_path, NULL);
			carLength = l;
			carWidth = w;
		};

		void update_state(dd time_step){
			state.x += getXDot()*time_step;
			state.y += getYDot()*time_step;
		}

		void control(){
			Control curControl;
			if (!path.empty()) {
				pthread_mutex_lock (&mutex_path);
				curControl = path.front();
				path.pop_front();
				pthread_mutex_unlock (&mutex_path);
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

		std::deque <Control>*  getPath()
		{
			return &path;
		}

		void setPath(std::deque <Control> &apath)
		{
			pthread_mutex_lock (&mutex_path);
			path = apath;
			pthread_mutex_unlock (&mutex_path);

		}

		void draw()
		{
			glColor3f(1.0,1.0,0.0);
			glPushMatrix();
			glTranslatef(state.x,state.y,0);
			glRotatef(state.theta*180.0/M_PI,0,0,1);
			glBegin(GL_QUADS);
			glVertex2f(-carLength/2,-carWidth/2);
			glVertex2f(carLength/2,-carWidth/2);
			glVertex2f(carLength/2,carWidth/2);
			glVertex2f(-carLength/2,carWidth/2);
			glEnd();
			glPopMatrix();
		}

	public:
		pthread_mutex_t mutex_path;

	private:
		dd carLength;
		dd carWidth;
		std::deque <Control> path;

};

#endif
