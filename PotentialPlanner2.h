#ifndef POTENTIAL_PLANNER2
#define POTENTIAL_PLANNER2

#include "Planner.h"
#include <pthread.h>

#define CHARGE 2.0

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
		PotentialPlanner2():m_charge(CHARGE) {};
		PotentialPlanner2(Car& acar, std::vector<Pedestrian*> &apedestrians):Planner(acar, apedestrians),m_charge(CHARGE),hlength(acar.getLength()/2),hwidth(acar.getWidth()/2){
			setVector2D(m_force,0.0,0.0);
		}

		//public functions
		void plan(std::vector<Pedestrian*> &apedestrians);
		void drawForce();


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
		dd m_charge;
		
		//private functions
		bool isDangerous(const State &astate);
		bool isVeryDangerous(const State &astate);
		bool isSemiDangerous(const State &astate);
		dd goalForce();
		Vector2D calcForce(Pedestrian &apedestrian);
		Vector2D calcForce(int i);
		void calcTotalForce();
		Control convertForceToControl(Vector2D f);

		//Car property (to shorten code)
		double hlength, hwidth; //half of length and width
};


#endif
