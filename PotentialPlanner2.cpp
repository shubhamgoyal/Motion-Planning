#include <cstdio>
#include <cassert>
#include "PotentialPlanner2.h"

#define DANGEROUS_Y_DIST 7.0
#define DANGEROUS_X_DIST 3.0
#define BUFFER_DIST 0.2

bool PotentialPlanner2::isDangerous(State astate)
{
	double hlength = car->getLength()/2;
	double hwidth = car->getWidth()/2;
	double dy = astate.y - car->getY();
	double dx = astate.x - car->getX();
	double dist = sqrt(dx*dx + dy*dy);
	double safetyBuffer = 1.6;
	double dangerZone = 50.0;
	if ( (dy > hlength && dy < dangerZone) || (dy < -hlength && dy < Y_MAX - dangerZone)) {
		dd x = astate.x, y = astate.y, v = astate.v, theta= astate.theta;
		//tt is the rough estimate on time needed for the car to 
		//reach the pedestrian y position
		dd tt = (y - car->getY())/car->getV();
		if (x < car->getX()+car->getWidth()/2 && v*cos(theta)>0.0001) {
			//We times 1.5 to consider the car deceleration
			if (x + tt*v*cos(theta)*safetyBuffer > car->getX() - car->getWidth()/2)
				return true;
		}
		if (x > car->getX()-car->getWidth()/2 && v*cos(theta)<0.0001) {
			if (x + tt*v*cos(theta)*safetyBuffer < car->getX() + car->getWidth()/2)
				return true;
		}
		if (x < car->getX()+car->getWidth()/2 && x > car->getX()-car->getWidth()/2 && y - car->getY() < DANGEROUS_Y_DIST) return true;

	}
	if (astate.y > car->getY()-car->getLength()/2 && dist < 2.0) return true;
	return false;
}

bool PotentialPlanner2::isSemiDangerous(State astate)
{
	double dx = astate.x - car->getX();
	double dy = astate.y - car->getY();
	double dist = sqrt(dx*dx + dy*dy);
	if (astate.y > car->getY() - car->getLength()/2 + BUFFER_DIST && dist < 4.0) return true;
	return false;
}

bool PotentialPlanner2::isVeryDangerous(State astate)
{
	//Very dangerous if would crash in 0.5s
	double dx = astate.x - car->getX();
	double dy = astate.y - car->getY();
	double theta = car->getTheta();
	double v = car->getV();
	//THE COORDINATE OF THE astate base on car coordinate system
	double dxx = dx*sin(theta) - dy*cos(theta);
	double dyy = dx*cos(theta) + dy*sin(theta);
	
	if (abs(dyy) < car->getWidth()/2 + BUFFER_DIST && dxx-car->getLength()/2-BUFFER_DIST < v/2)
	{
		return true;
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
	bool veryDangerous=false;
	bool semiDangerous=false;
	Vector2D resForce;
	dd dx = car->getX() - astate.x;
	dd dy = car->getY() - astate.y;
	dd dist = sqrt(dx*dx + dy*dy);
	dd y_factor = DANGEROUS_Y_DIST/dist;
	dd x_factor = DANGEROUS_X_DIST/(abs(dx)+0.001);
	if (isDangerous(astate))
	{
		danger = true;
		if (isVeryDangerous(astate))
		{
			veryDangerous = true;
		}

		apedestrian.setColor(1);
	}
	else if (isSemiDangerous(astate))
	{
		semiDangerous = true;
		apedestrian.setColor(3);
	}
	else
	{
		apedestrian.setColor(0);
	}
	dd forceVal = m_charge*car->getV()/(dist*dist);

	if (dy > -car->getLength()/2) forceVal=0.0;

	resForce.x = x_factor*forceVal*dx/dist;
	resForce.y = (forceVal*dy/dist)*(car->getV()/7.0)*(car->getV()/4.0);
	if (danger && !veryDangerous) 
	{
		resForce.x = -3.0*cos(astate.theta)*abs(resForce.x)*x_factor;

		bool tempAssert = (resForce.x*apedestrian.getXDot() <= 0.001);
		assert( tempAssert || (printf("----xforce: %lf, vxPed: %lf\n-----",resForce.x,apedestrian.getXDot() ),tempAssert));
		resForce.y *= 1e4*y_factor*y_factor;
		if (dist < 10) resForce.y *= x_factor*x_factor/6;
	}
	else if (veryDangerous)
	{
		resForce.x *= 1e5;
		resForce.y *= 1e5;
	}
	else if (semiDangerous)
	{
		resForce.x *= 3.0;
		resForce.y *= 3.0;
	}

	if (resForce.y > 0) resForce.y = 0;

	return resForce;
}



void PotentialPlanner2::calcTotalForce() 
{
	for (int i=0;i<pedestrians->size();++i)
	{
		m_force = addVector2D(m_force, calcForce( *((*pedestrians)[i]) ));
	}
	//ADD THE GOAL EFFECT
	m_force.y += 75.0*m_charge;

	//ADD THE ROAD EFFECT
	dd isInside = 1.0;
	dd dx_left = car->getX()-PAVEMENT_LEFT_X_MAX;
	dd dx_right = car->getX()-PAVEMENT_RIGHT_X_MIN;
	if (!(car->getX() > PAVEMENT_LEFT_X_MAX && car->getX() < PAVEMENT_RIGHT_X_MIN)) isInside=-1.0;
	m_force.x += (10.0*10.0)*isInside*m_charge*abs(car->getV()*cos(car->getTheta()))/(dx_left*dx_left*dx_left);
	m_force.x += (10.0*10.0)*isInside*m_charge*abs(car->getV()*cos(car->getTheta()))/(dx_right*dx_right*dx_right);

}

Control PotentialPlanner2::convertForceToControl(Vector2D f)
{
	dd norm1 = 1e-3;
	dd norm2 = 3e-4;
	dd maxAccel = 1e-2;
	if (car->getV() < 3) maxAccel = 5e-2;
	dd minAccel = -1.2e-1;
	//dd minAccel = -2e-2;
	dd maxRotate = 3e-4;
	dd maxAbsTheta = 1.5e-1;
	dd maxTheta = maxAbsTheta/(abs(car->getX()-(X_MAX+X_MIN)/2.0)+1.0);
	dd minTheta = -maxTheta;
	if (car->getX() - (X_MAX+X_MIN)/2.0 > 0) 
	{
		maxTheta = maxAbsTheta;
	}
	else
	{
		minTheta = -maxAbsTheta;
	}
	dd maxV = 25.0;
	
	Control c;
	dd theta = car->getTheta();
	c.h1 = norm1*(f.x*cos(theta) + f.y*sin(theta));
	c.h2 = norm2*(-f.x*sin(theta) + f.y*cos(theta))/car->getV();
	
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
	else if (c.h1 < minAccel) c.h1 = minAccel;

	if (car->getTheta() - M_PI/2.0 >  maxTheta && c.h2 > -1e-7) c.h2 = -1e-3;
	else if (car->getTheta() - M_PI/2.0 < minTheta && c.h2 < 1e-7) c.h2 = 1e-3;
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

void PotentialPlanner2::plan(std::vector<Pedestrian*> &apedestrians)
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


		
