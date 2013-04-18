#ifndef POTENTIAL_PLANNER2
#define POTENTIAL_PLANNER2

#include "Planner.h"

#define CHARGE 3.0

/* This planner will consider the repulsion force from
	the pedestrian and environment and convert it into control.
	For force in Y direction, there is a default field towards
	the goal, and we will consider only one pedestrian field 
	that is the maximum.
	**For temporary, we don't implement force to X direction**
	For force in X direction, we will sum up from all seen pedestrian
	and there will be a field quite similar to resistance in the road.
	*/

class PotentialPlanner2 : public Planner {
	public:
		//constructor
		PotentialPlanner2():m_charge(3.0) {};
		PotentialPlanner2(Car& acar, std::vector<Pedestrian*> apedestrians):Planner(acar, apedestrians),m_charge(3.0){}

		//public functions
		void plan(std::vector<Pedestrian*> &apedestrians);

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

		void setVector2D(Vector2D &v, dd x, dd y)
		{
			v.x = x;
			v.y = y;
		}

		Vector2D m_force;
		Control m_control;
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
		bool isVeryDangerous(State astate);
		bool isSemiDangerous(State astate);
		//Vector2D calcPartialForce(State astate);
		dd goalForce();
		Vector2D calcForce(Pedestrian &apedestrian);
		void calcTotalForce();
		Control convertForceToControl(Vector2D f);
		dd m_charge;

};


#endif
