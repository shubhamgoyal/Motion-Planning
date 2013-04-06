#ifndef PLANNER
#define PLANNER

#include <deque>
#include <vector>

#include "typeAndStruct.h"
#include "car.h"
#include "pedestrian.h"

class Planner {
	public:
		//Constructor
		Planner(Car* acar, std::vector<Pedestrian> *apedestrians){
			car = acar;
			pedestrians = apedestrians;
		};

		//Public functions
		std::deque<Control>* plan();

	protected:
		Car *car;
		std::vector<Pedestrian> *pedestrians;
		std::deque<Control> path;

};


#endif
