#include <deque>

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

		void update(){
			state.x += getXDot()*timeStep;
			state.y += getYDot()*timeStep;
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

	private:
		dd carLength = CARLENGTH;
		dd carWidth = CARWIDTH;
		std::deque <Control> path;
};
