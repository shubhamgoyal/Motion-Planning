#include "car.h"
#include "typeAndStruct.h"
#include "SimplePlanner.h"
#include "pedestrian.h"
#include "pedestrian_behavior.h"


#include <GL/glut.h>
#include <vector>
#include <cstdio>
#include <ctime>
#include <cstdlib>
#include <random>
#include <pthread.h>

int getRandomInt()
{
   static int isSeeded = 0;
   if (!isSeeded)
   {
      srand(time(NULL));
      isSeeded = 1;
   }
   return rand();
}

	State state;
	Car car;
	std::vector <Pedestrian> pedestrians;
	SimplePlanner planner;

static void Init(void)
{

    glClearColor(0.0, 0.0, 0.0, 0.0);
    glClearAccum(0.0, 0.0, 0.0, 0.0);
}

State getRandomPedestrianState()
{
   State st; 
   std::uniform_real_distribution<double> dist(0,1);
   std::mt19937 rng;
   rng.seed(std::random_device{}());
   if (getRandomInt()%2 == 0)
   {   
      st.x = PAVEMENT_LEFT_X_MIN + dist(rng)*(PAVEMENT_LEFT_X_MAX - PAVEMENT_LEFT_X_MIN);
   }   
   else
   {   
      st.x = PAVEMENT_RIGHT_X_MIN + dist(rng)*(PAVEMENT_RIGHT_X_MAX - PAVEMENT_RIGHT_X_MIN);
   }   
   
   st.y = PAVEMENT_LEFT_Y_MIN + dist(rng)*(PAVEMENT_LEFT_Y_MAX - PAVEMENT_LEFT_Y_MIN);
   st.v = 0;
   st.theta = 0;
	return st;
}


static void Reshape(int width, int height)
{

    glViewport(0, 0, width, height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
	 glTranslatef(-1,-1,0);
	 glScalef(1.0/20,1.0/500,1);
}

static void Draw(void)
{
	glClear(GL_COLOR_BUFFER_BIT);
	car.draw();
	for (int i=0;i<pedestrians.size();++i)
		pedestrians[i].draw();

	glutSwapBuffers();
	glFlush();
}

void* planning(void* args)
{
	while (1)
	{
		planner.plan(pedestrians);
	}
	return NULL;
}

void* updateStates(void* args)
{
	while (1)
	{
		planner.plan(pedestrians);
		for (int i=0;i<pedestrians.size();++i)
		{
			pedestrians[i].update_state(0.1);
		}
		car.control();
		car.update_state(0.1);
	}
	return NULL;
}

void UPDATESTATES()
{
	while (1)
	{
		planner.plan(pedestrians);
		for (int i=0;i<pedestrians.size();++i)
		{
			pedestrians[i].update_state(0.1);
		}
		car.control();
		car.update_state(0.1);
	}
}

static void update(int value)
{
	//if (car.getV() < 0.4) car.control(0.01,0.1);
	/*
	planner.plan(pedestrians);
	for (int i=0;i<pedestrians.size();++i)
	{
		pedestrians[i].update_state(0.1);
	}
	car.control();
	car.update_state(0.1);
	*/
	glutPostRedisplay();
	glutTimerFunc(20,update,0);
}

void GUI()
{

	//drawing
	//glutInit(&argc, argv);
	int test=0;
	glutInit(&test,NULL);
   glutInitDisplayMode(GLUT_RGB | GLUT_ACCUM | GLUT_DOUBLE);
   glutInitWindowSize(300, 500);
   glutCreateWindow("Accum Test");
   Init();
   glutReshapeFunc(Reshape);
   glutDisplayFunc(Draw);
	glutTimerFunc(20,update,0);
   glutMainLoop();
}

void* gui(void* args)
{

	//drawing
	//glutInit(&argc, argv);
	int test=0;
	glutInit(&test,NULL);
   glutInitDisplayMode(GLUT_RGB | GLUT_ACCUM | GLUT_DOUBLE);
   glutInitWindowSize(300, 500);
   glutCreateWindow("Accum Test");
   Init();
   glutReshapeFunc(Reshape);
   glutDisplayFunc(Draw);
	glutTimerFunc(20,update,0);
   glutMainLoop();
	return NULL;
}

int main(int argc, char **argv)
{
	state.x = 6;
	state.y = 0;
	state.v = 0;
	state.theta = M_PI/2;
/*
	State st;
	st.x = 1;
	st.y = 4;
	st.v = 0;
	st.theta = 0;

	State st2;
	st2.x = 13.0;
	st2.y = 100;
	st2.v = 0;
	st2.theta = 0;
*/
	car = Car(state,1.6,5);
	
	for (int i=0;i<NUMBER_OF_PEDESTRIANS;++i)
	{
		pedestrians.push_back(Pedestrian(getRandomPedestrianState(),Pedestrian_Behavior(),NUMBER_OF_TIMESTEPS));
	}	

	planner = SimplePlanner(car, pedestrians);
	pthread_t threadUpdate;
	pthread_t threadPlan;
	pthread_t threadGUI;
	//pthread_create(&threadUpdate,NULL,&updateStates,NULL);
	//pthread_create(&threadPlan, NULL, &planning, NULL);
	pthread_create(&threadGUI,NULL,&gui,NULL);
	
	UPDATESTATES();
	
	//GUI();


	return 0;
}
