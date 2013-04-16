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
	 glRotatef(90.0,0,0,1.0);
    glTranslatef(-0.5,-1,0);
    glScalef(50.0/height,50.0/width,1);
}

static void Draw(void)
{
   glClear(GL_COLOR_BUFFER_BIT);

	/*Draw environment*/
	glPushMatrix();
	glTranslatef(0,-car.getY() + 10,0);
	glColor3f(0.6,0.8,0.195);
	glBegin(GL_QUADS);
		glVertex2f(X_MIN,Y_MIN);
		glVertex2f(X_MIN,Y_MAX);
		glVertex2f(X_MAX,Y_MAX);
		glVertex2f(X_MAX,Y_MIN);
	glEnd();
	glColor3f(0.63,0.32,0.18);
	glBegin(GL_QUADS);
		glVertex2f(PAVEMENT_LEFT_X_MIN,Y_MIN);
		glVertex2f(PAVEMENT_LEFT_X_MIN,Y_MAX);
		glVertex2f(PAVEMENT_LEFT_X_MAX,Y_MAX);
		glVertex2f(PAVEMENT_LEFT_X_MAX,Y_MIN);
	glEnd();
	glBegin(GL_QUADS);
		glVertex2f(PAVEMENT_RIGHT_X_MIN,Y_MIN);
		glVertex2f(PAVEMENT_RIGHT_X_MIN,Y_MAX);
		glVertex2f(PAVEMENT_RIGHT_X_MAX,Y_MAX);
		glVertex2f(PAVEMENT_RIGHT_X_MAX,Y_MIN);
	glEnd();
	glColor3f(0.3,0.3,0.3);
	glBegin(GL_QUADS);
		glVertex2f(PAVEMENT_RIGHT_X_MIN,Y_MIN);
		glVertex2f(PAVEMENT_RIGHT_X_MIN,Y_MAX);
		glVertex2f(PAVEMENT_LEFT_X_MAX,Y_MAX);
		glVertex2f(PAVEMENT_LEFT_X_MAX,Y_MIN);
	glEnd();
	/*Draw static lines every 50m*/
	glColor3f(0.1,0.1,0.1);
	glBegin(GL_LINES);
		int numLine=10;
		for (int i=0;i<numLine;++i)
		{
			glVertex2f(X_MIN,Y_MIN+i*(float)(Y_MAX-Y_MIN)/numLine);
			glVertex2f(X_MAX,Y_MIN+i*(float)(Y_MAX-Y_MIN)/numLine);
		}
	glEnd();

	/*Draw object*/
   car.draw();
   for (int i=0;i<pedestrians.size();++i)
      pedestrians[i].draw();
	glPopMatrix();

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
   //glutInitWindowSize(300, 700);
   glutInitWindowSize(WINDOW_X_SIZE,WINDOW_Y_SIZE);
	glutCreateWindow("Accum Test");
   Init();
   glutReshapeFunc(Reshape);
   glutDisplayFunc(Draw);
   glutTimerFunc(1,update,0);
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
	clock_t before;
	double seconds;
	for (int i = 0; i < NUMBER_OF_TIMESTEPS; i++) {
		before = clock();
		//debug
		printf("%d, carV: %lf, carTheta: %lf\n",i,car.getV(), (car.getTheta()-M_PI/2)*180.0/M_PI);
		for (int j = 0; j < NUMBER_OF_PEDESTRIANS; j++) {
			pedestrians[j].update_state(TIME_STEP_DURATION);
		}
		car.control();
		car.update_state(TIME_STEP_DURATION);
		
		while ( (float)(clock()-before) < 1e-2*(CLOCKS_PER_SEC)  )
		{
		//	printf("diff: %e\n", (float)(after-before)/CLOCKS_PER_SEC);
		}
		

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
