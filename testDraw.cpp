#include "car.h"
#include "typeAndStruct.h"
#include "SimplePlanner.h"
#include "pedestrian.h"


#include <GL/glut.h>
#include <cstdio>
#include <vector>

	State state;
	Car car;
	std::vector <Pedestrian> pedestrians;
	SimplePlanner planner;

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
	 glScalef(1,0.333,1);
}

static void Draw(void)
{
	glClear(GL_COLOR_BUFFER_BIT);
	car.draw();

	glutSwapBuffers();
	glFlush();
}

static void update(int value)
{
	//if (car.getV() < 0.4) car.control(0.01,0.1);
	planner.plan(pedestrians);
	car.control();
	car.update(0.0001);
	glutPostRedisplay();
	glutTimerFunc(20,update,0);
}

int main(int argc, char **argv)
{
	state.x = 0;
	state.y = -3;
	state.v = 0;
	state.theta = M_PI/2;

	car = Car(state,0.16,0.5);

	planner = SimplePlanner(&car, pedestrians);

	//drawing
	glutInit(&argc, argv);
   glutInitDisplayMode(GLUT_RGB | GLUT_ACCUM | GLUT_DOUBLE);
   glutInitWindowSize(300, 1000);
   glutCreateWindow("Accum Test");
   Init();
   glutReshapeFunc(Reshape);
   glutDisplayFunc(Draw);
	glutTimerFunc(20,update,0);
   glutMainLoop();

	return 0;
}
