#include "SimplePlanner.h"
#include <cstdio>

double min(double a, double b)
{
	return a>b?b:a;
}

bool SimplePlanner::isDangerous(State astate)
{
   
	if ( astate.y > car->getY() + car->getLength()/2 && astate.y < car->getY()+50) {
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

bool SimplePlanner::existDangerous(std::vector<Pedestrian> &apedestrians)
{
	for (int i=0;i<apedestrians.size();++i)
	{
		if (isDangerous(apedestrians[i].getState()))
				return true;
	}
	return false;
}

void SimplePlanner::plan(std::vector<Pedestrian> &apedestrians){
	std::deque<Control>tempPath;
	//pedestrians = apedestrians;

	dd maxV = 10;
	dd maxTheta = 0.1;
	dd breakAccel = 1;
	dd speedAccel = 0.5;
	Control c;
	c.h1 = 0;
	c.h2 = 0;
	if (existDangerous(apedestrians))
	{
		c.h1 = -1*min(breakAccel,car->getV());
		/* debug */
		//printf("-------DANGEROUS!!!--------## v: %lf, decel: %lf\n",car->getV(),c.h1);
	}
	else if (car->getV() < maxV) c.h1 = speedAccel;
	else if (car->getV() >=maxV) c.h1=0;

	
	tempPath.push_back(c);
	pthread_mutex_lock(&car->mutex_path);
	path->swap(tempPath);
	pthread_mutex_unlock(&car->mutex_path);
	//car->setPath(path);
	
}
