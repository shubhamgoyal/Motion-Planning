#ifndef POTENTIAL_PLANNER
#define POTENTIAL_PLANNER

#include "Planner.h"

/* This planner will consider the repulsion force from
	the pedestrian and environment and convert it into control.
	For force in Y direction, there is a default field towards
	the goal, and we will consider only one pedestrian field 
	that is the maximum.
	**For temporary, we don't implement force to X direction**
	For force in X direction, we will sum up from all seen pedestrian
	and there will be a field quite similar to resistance in the road.
	*/

class PotentialPlanner : public Planner {
	public:
		//constructor
		PotentialPlanner(){};
		PotentialPlanner(Car& acar, std::vector<Pedestrian> apedestrians):Planner(acar, apedestrians){}

		//public functions
		void plan(std::vector<Pedestrian> &apedestrians);

	protected:

		//private struct and variable
		struct Vector2D {
			dd x;
			dd y;
		} ;

		struct Cell {
			dd x;
			dd y;
			dd q;
		} ;

		Vector2D addVector2D(Vector2D a, Vector2D b) {
			Vector2D c;
			c.x = a.x + b.x;
			c.y = a.y + b.y;
			return c;
		}

		Vector2D force;
		Control control;
		/* This is to implement force in the X direction.
			We need to discretize the pedestrian (many pedestrian
			in one place should be condsidered as one
			*/
		/*
		int cellIndex[2][500];
		vector <Cell> cells;
		*/
		
		//private functions
		bool isDangerous(State astate);
		//Vector2D calcPartialForce(State astate);
		dd dYForce(State astate);
		dd sYForce(State astate);
		dd goalForce();
		dd calcYForce(State astate);
		void calcTotalForce();
		Control convertForceToControl(Vector2D f);

};


#endif
