#include "Object.h"

class Car: public Object {
	public:
		Car():Object(){};
		Car(State astate):Object(astate){};
		Car(dd ax, dd ay, dd av, dd atheta):Object(ax,ay,av,atheta){};

		void update(){
			state.x += state.v*cos(state.theta)*timeStep;
	
	private:



};
