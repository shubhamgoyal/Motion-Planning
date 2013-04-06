#include <cmath>

#include "typeAndStruct.h"


class Object {
   protected:
      State state;
		dd timeStep;
   public:
   //Constructor
		Object () {
			state.x = 0;
			state.y = 0;
			state.v = 0;
			state.theta = 0;
		}

      Object (State astate) {
	 		state = astate;
      }
      
      Object (dd ax, dd ay, dd av, dd atheta) {
		  state.x = ax;
		  state.y = ay;
		  state.v = av;
		  state.theta = atheta;
	  	}	  

	//functions
		dd getX(){
			return state.x;
		}
		
		dd getY(){
			return state.y;
		}

		dd getV(){
			return state.v;
		}

		dd getXDot(){
			return state.v*cos(state.theta);
		}

		dd getYDot(){
			return state.v*sin(state.theta);
		}

		dd getTheta(){
			return state.theta;
		}

		State getState(){
			return state;
		}

		void setTimeStep(dd dt){
			timeStep = dt;
		}
};
