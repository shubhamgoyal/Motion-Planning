#include "SimplePlanner.h"

int min(double a, double b)
{
	return a>b?b:a;
}

bool SimplePlanner::isDangerous(State astate)
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

bool SimplePlanner::existDangerous()
{
	for (int i=0;i<pedestrians.size();++i)
	{
		if (isDangerous(pedestrians[i].getState()))
				return true;
	}
	return false;
}

void SimplePlanner::plan(std::vector<Pedestrian> apedestrians){
	pedestrians = apedestrians;
	dd maxV = 5;
	dd maxTheta = 0.1;
	Control c;
	c.h1 = 0;
	c.h2 = 0;
	if (existDangerous())
	{
		if (car->getV() > 0) c.h1 = -1*min(1,car->getV());
	}
	else
		if (car->getV() < maxV) c.h1 = 0.1;

	path.push_back(c);
	car->setPath(path);
	
}
