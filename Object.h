#ifndef OBJECT_HEADER
#define OBJECT_HEADER

#include "typeAndStruct.h"

class Object {
private:
	State state;

public:
	//Constructor
	Object () {
		state.x = 0;
		state.y = 0;
		state.xDot = 0;
		state.yDot = 0;
		state.theta = 0;
	}

   	Object (State astate) {
		state = astate;
    }
      
    Object (dd ax, dd ay, dd axDot, dd ayDot, dd atheta) {
		state.x = ax;
		state.y = ay;
		state.xDot = axDot;
		state.yDot = ayDot;
		state.theta = atheta;
	}	  

	//functions
	dd getX(){
		return state.x;
	}
		
	dd getY(){
		return state.y;
	}

	dd getXDot(){
		return state.xDot;
	}

	dd getYDot(){
		return state.yDot;
	}

	dd getTheta(){
		return state.theta;
	}

	State getState(){
		return state;
	}

	void update_state(double time_step);
};

#endif