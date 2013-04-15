#include "master.h"
#include <cstdio>
#include <ctime>
#include <cstdlib>
#include <random>
#include <GL/glut.h>

/*Functions for openGL */

static void Init(void)
{

    glClearColor(0.0, 0.0, 0.0, 0.0);
    glClearAccum(0.0, 0.0, 0.0, 0.0);
}

static void Reshape(int width, int height)
{

    glViewport(0, 0, width, height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(-1,-1,0);
    glScalef(1.0/10,1.0/300,1);
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

static void update(int value)
{
   glutPostRedisplay();
   glutTimerFunc(1,update,0);
}

void* gui(void* args) {
	/* debug */
	printf("### in GUI ###\n");
	//THE GUI
	int tempZero=0;
   glutInit(&tempZero, NULL);
   glutInitDisplayMode(GLUT_RGB | GLUT_ACCUM | GLUT_DOUBLE);
   glutInitWindowSize(300, 700);
   glutCreateWindow("Accum Test");
   Init();
   glutReshapeFunc(Reshape);
   glutDisplayFunc(Draw);
   glutTimerFunc(10,update,0);
   glutMainLoop();
	return NULL;
}

/*End of openGL functions*/

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

void initialize_environment() {
	printf("-----INITIALIZE-----\n");
	for (int i = 0; i < NUMBER_OF_PEDESTRIANS; i++) {
		printf("%d\n",i);
		pedestrians.push_back( *(new Pedestrian( getRandomPedestrianState(),*(new Pedestrian_Behavior()), NUMBER_OF_TIMESTEPS)));
	}
	State initialCarState = {(X_MAX - X_MIN)/2.0, CARLENGTH/2.0, 0.0, M_PI/2.0};
	car = Car(initialCarState, CARLENGTH, CARWIDTH);
	//planner = SimplePlanner(car, pedestrians);
	planner = PotentialPlanner2(car, pedestrians);
	//planner = PotentialPlanner(car, pedestrians);
}

void execute() {
	printf("-----EXECUTE-----\n");
	for (int i = 0; i < NUMBER_OF_TIMESTEPS; i++) {
		//debug
		printf("%d\n",i);
		for (int j = 0; j < NUMBER_OF_PEDESTRIANS; j++) {
			pedestrians[j].update_state(TIME_STEP_DURATION);
		}
		car.control();
		car.update_state(TIME_STEP_DURATION);
	}
	execute();
}

void* control(void* args) {
	//debug
	printf("---In control---\n");
	while (true) {
		planner.plan(pedestrians);
	}
	return NULL;
}


int main() {
	initialize_environment();
	pthread_t thread;
	pthread_t gui_thread;
	pthread_create(&thread, NULL, &control, NULL);
	pthread_create(&gui_thread, NULL, &gui, NULL /*(void*)something*/);
	execute();
}
