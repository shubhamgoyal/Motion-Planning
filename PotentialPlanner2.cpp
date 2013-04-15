#include <cstdio>
#include "PotentialPlanner2.h"

bool PotentialPlanner2::isDangerous(State astate)
{
	if ( astate.y > car->getY() + car->getLength()/2 && astate.y < car->getY()+100) {
		dd x = astate.x, y = astate.y, v = astate.v, theta= astate.theta;
		//tt is the rough estimate on time needed for the car to 
		//reach the pedestrian y position
		dd tt = (y - car->getY())/car->getV();
		if (x < car->getX()+car->getWidth()/2 && v*cos(theta)>0.0001) {
			//We times 1.5 to consider the car deceleration
			if (x + tt*v*cos(theta)*1.5 > car->getX() - car->getWidth()/2)
				return true;
		}
		if (x > car->getX()-car->getWidth()/2 && v*cos(theta)<0.0001) {
			if (x + tt*v*cos(theta)*1.5 < car->getX() + car->getWidth()/2)
				return true;
		}
	}
	return false;
}

dd PotentialPlanner2::goalForce()
{
	dd v = car->getV();
	dd maxGoalForce = 1e-1;
	dd c = 0.01;
	//Temporary force function
	return maxGoalForce;
}

PotentialPlanner2::Vector2D PotentialPlanner2::calcForce(Pedestrian &apedestrian)
{
	State astate = apedestrian.getState();
	bool danger=false;
	Vector2D resForce;
	dd dx = car->getX() - astate.x;
	dd dy = car->getY() - astate.y;
	dd dist = sqrt(dx*dx + dy*dy);
	if (isDangerous(astate))
	{
		danger = true;
		apedestrian.setColor(1);
	}
	else
	{
		apedestrian.setColor(0);
	}
	dd forceVal = m_charge*car->getV()/(dist*dist);
	if (dy > 0) forceVal=0.0;
	resForce.x = forceVal*dx/dist;
	resForce.y = forceVal*dy/dist;
	if (danger) 
	{
		resForce.x *= -1.0;
		resForce.y *= 1e3/dist;
	}
	return resForce;
}

void PotentialPlanner2::calcTotalForce() 
{
	for (int i=0;i<pedestrians->size();++i)
	{
		m_force = addVector2D(m_force, calcForce( (*pedestrians)[i]));
	}
	//ADD THE GOAL EFFECT
	m_force.y += 1.0;

	//ADD THE ROAD EFFECT
	dd isInside = 1.0;
	if (!(car->getX() > PAVEMENT_LEFT_X_MAX && car->getX() < PAVEMENT_RIGHT_X_MIN)) isInside=-1.0;
	m_force.x += isInside*1.0*abs(car->getV()*cos(car->getTheta()))/(car->getX()-PAVEMENT_LEFT_X_MAX);
	m_force.x += isInside*1.0*abs(car->getV()*cos(car->getTheta()))/(car->getX()-PAVEMENT_RIGHT_X_MIN);

}

Control PotentialPlanner2::convertForceToControl(Vector2D f)
{
	dd norm1 = 1e-3;
	dd norm2 = 1e-5;
	dd maxAccel = 1e-3;
	dd maxRotate = 1e-3;
	dd maxTheta = 1e-1;
	dd maxV = 15.0;
	
	Control c;
	dd theta = car->getTheta();
	c.h1 = norm1*(f.x*cos(theta) + f.y*sin(theta));
	c.h2 = norm2*(-f.x*sin(theta) + f.y*cos(theta));
	
	/* DEBUGGING *//*	
	static unsigned int count=0;
	static unsigned int count2=0;
	
	if (car->getV() <= 1e-5 && car->getV() >= -1e-5)
	{
		if (count <= 1000) count++;
	}
	

	if (car->getV() <= 1e-5 && car->getV() >= -1e-5 && (count > 1000 || count2 >100))
	{
		printf("v: %lf, theta: %lf",car->getV(), car->getTheta()-M_PI/2.0);
		printf("\th1: %lf, h2: %lf, \t f.x= %lf, f.y= %lf\n", c.h1, c.h2,f.x,f.y);
		count = 0;
		count2++;
	}

	if (car->getTheta() - M_PI/2.0 >  maxTheta && c.h2 > -1e-7) printf("----VERY BIG POSITIVE THETA----\n");
	else if (car->getTheta() - M_PI/2.0 < -maxTheta && c.h2 < 1e-7) printf("----VERY BIG NEGATIVE THETA----\n");
	else if (c.h2 > maxRotate) printf("----VERY BIG POSITIVE ROTATION----");
	else if (c.h2 < -maxRotate) c.h2 = -maxRotate;
	

	/* END DEBUGGIN */

	if ((car->getV() > maxV && c.h1 > 1e-5) || (car->getV() < -maxV && c.h1 < -1e-5)) c.h1 = 0;
	else if (c.h1 > maxAccel) c.h1 = maxAccel;
	else if (c.h1 < -maxAccel) c.h1 = -maxAccel;

	if (car->getTheta() - M_PI/2.0 >  maxTheta && c.h2 > -1e-7) c.h2 = -1e-3;
	else if (car->getTheta() - M_PI/2.0 < -maxTheta && c.h2 < 1e-7) c.h2 = 1e-3;
	else if (c.h2 > maxRotate) c.h2 = maxRotate;
	else if (c.h2 < -maxRotate) c.h2 = -maxRotate;
	
	/*DEBUGGING*//*
	if (car->getV() <= 1e-5 && car->getV() >= -1e-5)
	{
		if (count <= 1000) count++;
	}
	else 
	{
		count=0;
		count2=0;
	}
	/**********/

	return c;
}

void PotentialPlanner2::plan(std::vector<Pedestrian> &apedestrians)
{
	std::deque<Control> tempPath;

	pedestrians = &apedestrians;
	setVector2D(m_force,0.0,0.0);
	calcTotalForce();
	tempPath.push_back(convertForceToControl(m_force));
	//path.push_back(convertForceToControl(force));
	pthread_mutex_lock(&car->mutex_path);
	//car->setPath(path);
	path->swap(tempPath);
	pthread_mutex_unlock(&car->mutex_path);

}


		
