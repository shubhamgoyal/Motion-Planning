#ifndef SIMPLE_PLANNER
#define SIMPLE_PlANNER

#include "Planner.h"

class SimplePlanner:public Planner {
	public:
		//constructor
		SimplePlanner(){};
		SimplePlanner(Car &acar, std::vector<Pedestrian> &apedestrians):Planner(acar,apedestrians){}

		//public function
		void plan(std::vector<Pedestrian> &apedestrians);
	protected:
		bool isDangerous(State astate);
		bool existDangerous(std::vector<Pedestrian> &apedestrians);
		//bool existDangerous();

};




#endif
