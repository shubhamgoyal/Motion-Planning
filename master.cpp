#include "master.h"

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <GL/glut.h>


/*Functions for openGL */

static void Init(void)
{

    glClearColor(0.0, 0.0, 0.0, 0.0);
    glClearAccum(0.0, 0.0, 0.0, 0.0);
}

/* To see 50m ahead *//*
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
/*******************/

/* To see the overview of all road (to see pedestrian behavior) */
static void Reshape(int width, int height)
{

    glViewport(0, 0, width, height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
	 glRotatef(90.0,0,0,1.0);
    glTranslatef(-0.5,-0.8,0);
    glScalef(20.0/height,7.0/width,1);
}
/********************/
static void drawEnv(double scaleX, double scaleY, double dx, double dy)
{
	/*Draw environment*/
	glPushMatrix();
	glScalef(scaleX, scaleY, 1);
	glTranslatef(dx,dy,0);
	//BACKGROUND GRASS
	glColor3f(0.6,0.8,0.195);
	glBegin(GL_QUADS);
		glVertex2f(X_MIN,Y_MIN);
		glVertex2f(X_MIN,Y_MAX);
		glVertex2f(X_MAX,Y_MAX);
		glVertex2f(X_MAX,Y_MIN);
	glEnd();

	//BACKGROUND PAVEMENT
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

	//ROAD
	glColor3f(0.3,0.3,0.3);
	glBegin(GL_QUADS);
		glVertex2f(PAVEMENT_RIGHT_X_MIN,Y_MIN);
		glVertex2f(PAVEMENT_RIGHT_X_MIN,Y_MAX);
		glVertex2f(PAVEMENT_LEFT_X_MAX,Y_MAX);
		glVertex2f(PAVEMENT_LEFT_X_MAX,Y_MIN);
	glEnd();

	//ZEBRA CROSS
	glColor3f(0.75,0.9,0.9);
	glBegin(GL_QUADS);
		glVertex2f(PAVEMENT_RIGHT_X_MIN,ZEBRA1_Y_MIN);
		glVertex2f(PAVEMENT_RIGHT_X_MIN,ZEBRA1_Y_MAX);
		glVertex2f(PAVEMENT_LEFT_X_MAX,ZEBRA1_Y_MAX);
		glVertex2f(PAVEMENT_LEFT_X_MAX,ZEBRA1_Y_MIN);
	glEnd();
	glColor3f(0.75,0.9,0.9);
	glBegin(GL_QUADS);
		glVertex2f(PAVEMENT_RIGHT_X_MIN,ZEBRA2_Y_MIN);
		glVertex2f(PAVEMENT_RIGHT_X_MIN,ZEBRA2_Y_MAX);
		glVertex2f(PAVEMENT_LEFT_X_MAX,ZEBRA2_Y_MAX);
		glVertex2f(PAVEMENT_LEFT_X_MAX,ZEBRA2_Y_MIN);
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

}



static void Draw(void)
{
   glClear(GL_COLOR_BUFFER_BIT);

	/*Draw environment*//*
	glPushMatrix();
	//glTranslatef(0,-car.getY() + 10,0);
	//BACKGROUND GRASS
	glColor3f(0.6,0.8,0.195);
	glBegin(GL_QUADS);
		glVertex2f(X_MIN,Y_MIN);
		glVertex2f(X_MIN,Y_MAX);
		glVertex2f(X_MAX,Y_MAX);
		glVertex2f(X_MAX,Y_MIN);
	glEnd();

	//BACKGROUND PAVEMENT
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

	//ROAD
	glColor3f(0.3,0.3,0.3);
	glBegin(GL_QUADS);
		glVertex2f(PAVEMENT_RIGHT_X_MIN,Y_MIN);
		glVertex2f(PAVEMENT_RIGHT_X_MIN,Y_MAX);
		glVertex2f(PAVEMENT_LEFT_X_MAX,Y_MAX);
		glVertex2f(PAVEMENT_LEFT_X_MAX,Y_MIN);
	glEnd();

	//ZEBRA CROSS
	glColor3f(0.75,0.9,0.9);
	glBegin(GL_QUADS);
		glVertex2f(PAVEMENT_RIGHT_X_MIN,ZEBRA1_Y_MIN);
		glVertex2f(PAVEMENT_RIGHT_X_MIN,ZEBRA1_Y_MAX);
		glVertex2f(PAVEMENT_LEFT_X_MAX,ZEBRA1_Y_MAX);
		glVertex2f(PAVEMENT_LEFT_X_MAX,ZEBRA1_Y_MIN);
	glEnd();
	glColor3f(0.75,0.9,0.9);
	glBegin(GL_QUADS);
		glVertex2f(PAVEMENT_RIGHT_X_MIN,ZEBRA2_Y_MIN);
		glVertex2f(PAVEMENT_RIGHT_X_MIN,ZEBRA2_Y_MAX);
		glVertex2f(PAVEMENT_LEFT_X_MAX,ZEBRA2_Y_MAX);
		glVertex2f(PAVEMENT_LEFT_X_MAX,ZEBRA2_Y_MIN);
	glEnd();

	/*Draw static lines every 50m*//*
	glColor3f(0.1,0.1,0.1);
	glBegin(GL_LINES);
		int numLine=10;
		for (int i=0;i<numLine;++i)
		{
			glVertex2f(X_MIN,Y_MIN+i*(float)(Y_MAX-Y_MIN)/numLine);
			glVertex2f(X_MAX,Y_MIN+i*(float)(Y_MAX-Y_MIN)/numLine);
		}
	glEnd();

	/*Draw object*//*
   car.draw();
   for (int i=0;i<pedestrians.size();++i)
      pedestrians[i].draw();
	glPopMatrix();
	*/
	
	//Draw overview:
	drawEnv(1.0,1.0,-15.0,-10.0);
	
	//Draw 2 lines that always changes
	double shiftLength = (Y_MAX-Y_MIN)/5.0;
	double shownLength = (Y_MAX-Y_MIN)*3.0/10.0;
	static double curShift1 = 0;
	static double curShift2 = shiftLength;

	if ( !(car.getY() - curShift1 >= Y_MAX-2*shiftLength) && car.getY() >= curShift1 - 2*shiftLength && car.getY() - curShift1 > shownLength) 
	{
		curShift1 += 2*shiftLength;
		if (curShift1 >= Y_MAX-1.0 && curShift1 <= Y_MAX+1.0) 
		{
			curShift1 = 0.0;
		}
	}
	if ( !(car.getY() - curShift2 >= Y_MAX-2*shiftLength) && car.getY() >= curShift2 - 2*shiftLength && car.getY() - curShift2 > shownLength) 
	{
		curShift2 += 2*shiftLength;
		if (curShift2 >= Y_MAX-1.0 && curShift2 <= Y_MAX+1.0) 
		{
			curShift2 = 0.0;
		}
	}
	if (car.getY() < curShift1 - 2*shiftLength && car.getY() + (Y_MAX-Y_MIN) - curShift1 > shownLength) curShift1 += 2*shiftLength - (Y_MAX - Y_MIN);
	if (car.getY() < curShift2 - 2*shiftLength && car.getY() + (Y_MAX-Y_MIN) - curShift2 < shownLength ) curShift2 += 2*shiftLength - (Y_MAX - Y_MIN);
	drawEnv(1.2,3.0,40,-5-curShift1);
	drawEnv(1.2,3.0,20,-5-curShift2);

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
		srand((int)((long long int)RANDOM_SEED*58542%(1<<31-1)));
		isSeeded = 1;
	}
	return rand();
}

State getRandomPedestrianState()
{
	State st;
	std::uniform_real_distribution<double> dist(0,1);
	std::mt19937 rng;
	//rng.seed(std::random_device{}());
	rng.seed(getRandomInt());
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
	State initialCarState = {(X_MAX - X_MIN)/2.0, CARLENGTH/2.0, 0.0, M_PI/2.0};
	car = Car(initialCarState, CARLENGTH, CARWIDTH);
	previousCarState = initialCarState;
	for (int i = 0; i < NUMBER_OF_PEDESTRIANS; i++) {
		printf("%d\n",i);
		pedestrians.push_back( *(new Pedestrian( getRandomPedestrianState(),*(new Pedestrian_Behavior(car)), NUMBER_OF_TIMESTEPS)));
		if (pedestrians[i].getY() < 100) seenPedestrians.push_back(&pedestrians[i]);
	}
	//planner = SimplePlanner(car, pedestrians);
	planner = PotentialPlanner2(car, seenPedestrians);
	//planner = PotentialPlanner(car, pedestrians);
}

bool isPointInsideCar(State carState, dd x, dd y)
{
	dd hwidth = CARWIDTH/2.0 + 0.1;
	dd hlength= CARLENGTH/2.0 + 0.1;
	dd cx = carState.x;
	dd cy = carState.y;
	dd theta = carState.theta;
	dd dx1 = hlength*cos(theta) - hwidth*sin(theta);
	dd dx2 = hlength*cos(theta) + hwidth*sin(theta);
	dd dy1 = hlength*sin(theta) + hwidth*cos(theta);
	dd dy2 = hlength*sin(theta) - hwidth*cos(theta);
	//Check all the cross product
	if ( (cx + dx1 - x)*(cy + dy2 - y) - (cx + dx2 - x)*(cy + dy1 - y) > -0.001) return false;
	if ( -(cx - dx1 - x)*(cy + dy2 - y) + (cx + dx2 - x)*(cy - dy1 - y) > -0.001) return false;
	if ( (cx - dx1 - x)*(cy - dy2 - y) - (cx - dx2 - x)*(cy - dy1 - y) > -0.001) return false;
	if ( -(cx + dx1 - x)*(cy - dy2 - y) + (cx - dx2 - x)*(cy + dy1 - y) > -0.001) return false;
	return true;
	
}
bool carHitPedestrian(State previousCarState, State currentCarState, State pedestrianState)
{
	dd pedX = pedestrianState.x, pedY = pedestrianState.y;
	bool curInside = isPointInsideCar(currentCarState, pedX, pedY);
	bool prevInside = isPointInsideCar(previousCarState, pedX, pedY);
	
	/* DEBUGGING */
	//if (curInside) {printf("\n---HELLO1\n");exit(0);}
	//if (prevInside) {printf("\n---HELLO2---\n");exit(0);}
	/*************/
	
	if ( curInside && !prevInside)
	{
		dd prevCarY = previousCarState.y, pedY = pedestrianState.y;
		if (pedY > prevCarY + CARLENGTH/2*cos(previousCarState.theta) - 0.1) return true;
	}
	return false;
}

void execute() {
	printf("-----EXECUTE-----\n");
	clock_t before;
	static unsigned int count = 0;
	for (int i = 0; i < NUMBER_OF_TIMESTEPS; i++) {
		before = clock();
		timeFromStart = (float)((double)(clock()-start)/CLOCKS_PER_SEC/2.0);
		count++;
		//debug
		if (count >= 15)
		{
			time_t now;
			time(&now);
			printf("%d, car: x=%lf, yTot=%lf, V= %lf, Theta: %lf\n NumCollide: %u, time: %f, time2: %lf\n",i,car.getX(), yTotal, car.getV(), (car.getTheta()-M_PI/2)*180.0/M_PI, numCollision, timeFromStart, difftime(now,start2));
			count=0;
		}
		
		for (int j = 0; j < NUMBER_OF_PEDESTRIANS; j++) {
			pedestrians[j].update_state(TIME_STEP_DURATION);

		}
		car.control();
		car.update_state(TIME_STEP_DURATION);

		/*Checking car -- pedestrian collision (using previousCarState)
		 *Also manage the seenPedestrian */
		seenPedestrians.clear();
		for (int j=0; j< NUMBER_OF_PEDESTRIANS; j++) {
			if (carHitPedestrian(previousCarState, car.getState(), pedestrians[j].getState()))
					numCollision++;
			if (pedestrians[j].getY() < car.getY() + 100)
			{
				if (pedestrians[j].getY() > car.getY()-car.getLength() || pedestrians[j].getY() < (car.getY()-400))
				{
					seenPedestrians.push_back(&pedestrians[j]);
					pedestrians[j].setColor(0);
				}
				else pedestrians[j].setColor(2);
			}
			else pedestrians[j].setColor(2);
		}
		
		/* Calculating yTotal */
		if (!(car.getY() + (Y_MAX - Y_MIN)/2.0 < previousCarState.y)) yTotal += car.getY() - previousCarState.y;
		else yTotal += car.getY() + (Y_MAX - Y_MIN) - previousCarState.y;



		previousCarState = car.getState();
		
		while ( (float)(clock()-before) < 1e-2*(CLOCKS_PER_SEC)  )
		{
		//	printf("diff: %e\n", (float)(after-before)/CLOCKS_PER_SEC);
		}
		

	}
	execute();
}

/*
void* control(void* args) {
	//debug
	printf("---In control---\n");
	while (true) {
		planner.plan(pedestrians);
	}
	return NULL;
}
*/

void* control(void* args) {
	//debug
	printf("---In control---\n");
	while (true) {
		planner.plan(seenPedestrians);
	}
	return NULL;
}

int main() {
	initialize_environment();
	pthread_t thread;
	pthread_t gui_thread;
	start = clock();
	time(&start2);
	pthread_create(&thread, NULL, &control, NULL);
	pthread_create(&gui_thread, NULL, &gui, NULL /*(void*)something*/);
	execute();
}
