#include "PotentialPlanner.h"

bool PotentialPlanner::isDangerous(State astate)
{
	if ( state.y > car->getY() + car->getLength/2) {
		dd x = astate.x, y = astate.y, v = astate.v, theta= astate.theta;
		//tt is the rough estimate on time needed for the car to 
		//reach the pedestrian y position
		dd tt = (y - car->getY())/car->getV();
		if (x < car->getX()+car->getWidth()/2 && v*cos(theta)>0) {
			//We times 1.5 to consider the car deceleration
			if (x + tt*v*cos(theta)*1.5 > car->getX - car->getWidth()/2)
				return true;
		}
		if (x > car->getX()-car->getWidth()/2 && v*cos(theta)<0) {
			if (x + tt*v*cos(theta)*1.5 < car->getX + car->getWidth()/2)
				return true;
		}
	}
	return false;
}



		
