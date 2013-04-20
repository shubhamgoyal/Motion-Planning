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
		Planner(){};

		Planner(Car &acar, std::vector<Pedestrian*> &apedestrians){
			car = &acar;
			path = car->getPath();
			pedestrians = &apedestrians;
		};

		//Public functions
		/* plan function will update the list of control from the 
			car class (car->setPath()). 
		 */
		void plan(std::vector<Pedestrian*> &apedestrians);

	protected:
		Car *car;
		std::vector<Pedestrian*> *pedestrians;
		std::deque<Control> *path;

};


#endif
