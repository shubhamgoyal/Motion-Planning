#ifndef POTENTIAL_PLANNER
#define POTENTIAL_PLANNER

#include "Planner.h"

class PotentialPlanner : public Planner {
	public:
		//constructor
		PotentialPlanner(Car* acar, std::vector<Pedestrian>* apedestrians):Planner(acar, apedestrians){}


};


#endif
