#include <cstdio>
#include "PotentialPlanner.h"

bool PotentialPlanner::isDangerous(State astate)
{
	if ( astate.y > car->getY() + car->getLength()/2) {
		dd x = astate.x, y = astate.y, v = astate.v, theta= astate.theta;
		//tt is the rough estimate on time needed for the car to 
		//reach the pedestrian y position
		dd tt = (y - car->getY())/car->getV();
		if (x < car->getX()+car->getWidth()/2 && v*cos(theta)>0) {
			//We times 1.5 to consider the car deceleration
			if (x + tt*v*cos(theta)*1.5 > car->getX() - car->getWidth()/2)
				return true;
		}
		if (x > car->getX()-car->getWidth()/2 && v*cos(theta)<0) {
			if (x + tt*v*cos(theta)*1.5 < car->getX() + car->getWidth()/2)
				return true;
		}
	}
	return false;
}

dd PotentialPlanner::dYForce(State astate)
{
	dd dist = astate.y - car->getY() - car->getLength()/2;
	dd v = car->getV();
	dd c = 1.0;
	//Just for temporary force function
	return c*v*v/dist/dist;
	/* We need a function such that given any v and dist, it will stop
		at same distance 
		*/
}

dd PotentialPlanner::sYForce(State astate)
{
	dd dist = astate.y - car->getY() - car->getLength()/2;
	dd v = car->getV();
	dd c = 0.01; //just some adjustable constant
	//Just for temporary force function
	return c*v;
	/*We need a function such that the maximum speed of the car
	  when there is a pedestrian ahead is limited this speed is 
	  much safer than the maximum speed when there is no 
	  pedestrian ahead
	  */

}

dd PotentialPlanner::goalForce()
{
	dd v = car->getV();
	dd maxGoalForce = 10;
	dd c = 0.01;
	//Temporary force function
	return maxGoalForce;
}

dd PotentialPlanner::calcYForce(State astate)
{
	if (isDangerous(astate))
		return dYForce(astate); //dangerous Y force
	else
		return sYForce(astate); //safe Y force (walk in the pavement)
}

void PotentialPlanner::calcTotalForce() 
{
	/*Currently we only implement the force in Y direction */
	//get the pedestrian that give max force in Y direction
	dd maxYForce = 0;
	int maxInd = -1;
	for (int i=0;i<pedestrians->size();++i)
	{
		dd yForce = calcYForce((*pedestrians)[i].getState());
		if(maxYForce < yForce)
		{
			maxYForce = yForce;
			maxInd = i;
		}
	}

	force.x = 0;
	force.y = goalForce() - maxYForce;

}

Control PotentialPlanner::convertForceToControl(Vector2D f)
{
	//For temporary only. Only consider y direction.
	Control c;
	c.h1 = force.y/1000;
	c.h2 = 0;
	return c;
}

void PotentialPlanner::plan(std::vector<Pedestrian> &apedestrians)
{
	std::deque<Control> tempPath;

	pedestrians = &apedestrians;
	calcTotalForce();
	tempPath.push_back(convertForceToControl(force));
	//path.push_back(convertForceToControl(force));
	pthread_mutex_lock(&car->mutex_path);
	//car->setPath(path);
	path->swap(tempPath);
	pthread_mutex_unlock(&car->mutex_path);

}


		
