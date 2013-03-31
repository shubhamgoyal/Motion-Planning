#include <typeAndStruct.h>


class 2DObject {
   private:
      2DState state;
   public:
   //Constructor
		2DObject () {
			state.x = 0;
			state.y = 0;
			state.xDot = 0;
			state.yDot = 0;
			state.theta = 0;
		}

      2DObject (2DState astate) {
	 		state = astate;
      }
      
      2DObject (dd ax, dd ay, dd axDot, dd ayDot, dd atheta) {
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

		2DState getState(){
			return state;
		}
};
