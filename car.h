#ifndef CAR
#define CAR
#include <deque>
#include <GL/glut.h>
#include <cstdio>
//#define _USE_MATH_DEFINES
//#include <cmath>

#include "environment.h"
#include "Object.h"
#include <pthread.h>
#include <iostream>
//#define CARLENGTH 3.0
//#define CARWIDTH 1.6


class Car: public Object {
	public:
		Car():Object(), timeStop(0), isHorn(false){
			pthread_mutex_init(&mutex_path, NULL);
			carLength = CARLENGTH;
			carWidth = CARWIDTH;
		}

		Car(dd l, dd w):Object(),timeStop(0), isHorn(false){
			pthread_mutex_init(&mutex_path, NULL);
			carLength = l;
			carWidth = w;
		};

		Car(State astate, dd l, dd w):Object(astate), timeStop(0), isHorn(false){
			pthread_mutex_init(&mutex_path, NULL);
			carLength = l;
			carWidth = w;
		};

		Car(State astate):Object(astate), timeStop(0), isHorn(false){
			pthread_mutex_init(&mutex_path, NULL);
			carLength = CARLENGTH;
			carWidth = CARWIDTH;
		};

		Car(dd ax, dd ay, dd av, dd atheta, dd l, dd w):Object(ax,ay,av,atheta), timeStop(0), isHorn(false){
			pthread_mutex_init(&mutex_path, NULL);
			carLength = l;
			carWidth = w;
		};

		void update_state(dd time_step){
			if (HORN_ENABLED)
			{
				if (state.v < 1e-5 && state.v > -1e-5)
				{
					timeStop++;
					if (timeStop > WAIT_TO_HORN)
					{
						std::cout << '\a' << '\a';
						isHorn = true;
						timeStop -= HORN_INTERVAL;
					}
				}
				else
				{
					timeStop = 0;
					isHorn = false;
				}
			}
			state.x += getXDot()*time_step;
			state.y += getYDot()*time_step;
			if (state.y >= Y_MAX) state.y -= Y_MAX;
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
				curControl.h1 = 0.0;
				curControl.h2 = 0.0;
			}

			state.v += curControl.h1;
			if ((state.v > 1e-6) && (curControl.h2 > 1e-8 || curControl.h2 < -1e-8) )
				state.theta += state.v*tan(curControl.h2)/carLength;
			else state.theta = state.theta;

			if (state.v < 0) state.v = 0;
		}

		void control(dd h1, dd h2){
			state.v += h1;
			state.theta += state.v*tan(h2)/carLength/10;
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

		bool getHorn()
		{
			return isHorn;
		}

		bool setHornOff()
		{
			isHorn = false;
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
			double hlength = carLength/2-0.1;
			double hwidth = carWidth/2-0.1;
			glVertex2f(-hlength,-hwidth);
			glVertex2f( hlength,-hwidth);
			glVertex2f( hlength, hwidth);
			glVertex2f(-hlength, hwidth);
			glEnd();
			glPopMatrix();
		}

	public:
		pthread_mutex_t mutex_path;

	private:
		dd carLength;
		dd carWidth;
		unsigned int timeStop;
		bool isHorn;
		std::deque <Control> path;

};

#endif
