#include "Object.h"

class Car: public Object {
	public:
		Car():Object(){
			carLength = 5;
			carWidth = 2;
		}

		Car(dd l, dd w):Object(){
			carLength = l;
			carWidth = w;
		};

		Car(State astate, dd l, dd w):Object(astate){
			carLength = l;
			carWidth = w;
		};

		Car(State astate):Object(astate){
			carLength = 5;
			carWidth = 2;
		};

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
			state.theta += tan(h2)/carLength;
		}

	private:
		dd carLength;
		dd carWidth;
};
