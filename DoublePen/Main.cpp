#include <GL/glut.h>
#include "Mesh.h"
#include "Sphere.h"
#include "Simulation.h"
#include "matrix.h"
#include "vec3.h"

#include <thread>
#include <mutex>
#include <vector>
#include <stdlib.h>

using namespace std;

double ballx, bally, ballz;  //The position of the ball - you can set this in your code
double boxxl, boxxh, boxyl, boxyh, boxzl, boxzh;  // The low and high x, y, z values for the box sides

double ballx2, bally2, ballz2;

int rotateon;

double xmin, xmax, ymin, ymax, zmin, zmax;
double maxdiff;

int lastx, lasty;
int xchange, ychange;
float spin = 0.0;
float spinup = 0.0;

GLfloat qaAmbientLight[] = { 0.2, .2, 0.2, 1.0 };
GLfloat qaDiffuseLight[] = { 1, 1, 1, 1.0 };
GLfloat qaSpecularLight[] = { .7, .7, .7, 1.0 };

GLfloat qaLightPosition[] = { 50, 0, 50, 1 };
GLfloat qaLightPosition1[] = { -50, 0, -50, 1 };

Simulation* simulation = nullptr;

void DrawCircle(float cx, float cy, float r, int num_segments)
{
	float theta = 2 * 3.1415926 / float(num_segments);
	float tangetial_factor = tanf(theta);//calculate the tangential factor 

	float radial_factor = cosf(theta);//calculate the radial factor 

	float x = r;//we start at angle = 0 

	float y = 0;

	glBegin(GL_LINE_LOOP);
	glColor3f(0, 0, 0);
	for (int ii = 0; ii < num_segments; ii++)
	{
		glVertex3f(x + cx, -95, y + cy);//output vertex 

								   //calculate the tangential vector 
								   //remember, the radial vector is (x, y) 
								   //to get the tangential vector we flip those coordinates and negate one of them 

		float tx = -y;
		float ty = x;

		//add the tangential vector 

		x += tx * tangetial_factor;
		y += ty * tangetial_factor;

		//correct using the radial factor 

		x *= radial_factor;
		y *= radial_factor;
	}
	glEnd();
}

void display(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glPushMatrix();

	//rotate the view
	glRotatef(spinup, 1.0, 0.0, 0.0);
	glRotatef(spin, 0.0, 1.0, 0.0);
	glLightfv(GL_LIGHT0, GL_POSITION, qaLightPosition);
	glLightfv(GL_LIGHT1, GL_POSITION, qaLightPosition1);

	glBegin(GL_QUADS);

	GLfloat green[] = { 0, 1.0, 0, 1 };
	GLfloat red[] = { 1.0, 0, 0, 1 };
	GLfloat white[] = { 1, 1.0, 1, 1 };

	GLfloat whitesoft[] = { .4, .4, .4, 1 };

	std::lock_guard<std::mutex> lock(simulation->lock);

	for (int i = 0; i < simulation->planes.size(); i++)
	{
		//if (i % 2 == 0)
		//	glColor3f(.9, .9, .9);
		//else
		//	glColor3f(.2, .2, .2);

		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, whitesoft);

		glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, whitesoft);

		glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 2);

		glNormal3f(0, 1, 0);
		glVertex3f(simulation->planes[i].vertices[0].x(), simulation->planes[i].vertices[0].y(), simulation->planes[i].vertices[0].z());
		glVertex3f(simulation->planes[i].vertices[1].x(), simulation->planes[i].vertices[1].y(), simulation->planes[i].vertices[1].z());
		glVertex3f(simulation->planes[i].vertices[2].x(), simulation->planes[i].vertices[2].y(), simulation->planes[i].vertices[2].z());
		glVertex3f(simulation->planes[i].vertices[3].x(), simulation->planes[i].vertices[3].y(), simulation->planes[i].vertices[3].z());
	}

	glEnd();

	glMatrixMode(GL_MODELVIEW);

	if (simulation->showballs)
	{

		for (int i = 0; i < simulation->meshes.size(); i++)
		{
			if (i % 2 == 0)
			{
				glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, green);

				glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, white);

				glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 9);

			}
			else
			{
				GLfloat blue[] = { 0, 0.3, 1.0, 1 };
				glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, blue);

				glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, white);

				glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 9);
			}

			Mesh* mesh = simulation->meshes[i];
			auto pos = mesh->getX();

			glColor3f(1, 0, 0);
			glPushMatrix();
			glTranslatef(pos.x(), pos.y(), pos.z());
			glutSolidSphere(12, 50, 50);
			glPopMatrix();
		}
		glDisable(GL_LIGHTING);
		Mesh* mesh = simulation->meshes[0];
		auto pos = mesh->getX();
		mesh = simulation->meshes[1];
		auto pos1 = mesh->getX();
		glColor3f(1, 1, 1);
		glBegin(GL_LINES);
		glVertex3f(0, 120, 0);
		glVertex3f(pos.x(), pos.y(), pos.z());
		glVertex3f(pos.x(), pos.y(), pos.z());
		glVertex3f(pos1.x(), pos1.y(), pos1.z());
		glEnd();
		glEnable(GL_LIGHTING);

		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, red);

		glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, white);

		glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 9);
		glPushMatrix();
		glTranslatef(0, 120, 0);
		glutSolidSphere(5, 50, 50);
		glPopMatrix();
	}

	

	glDisable(GL_LIGHTING);

	if (simulation->draw1)
	{
		glBegin(GL_LINES);
		glLineWidth(6);
		glColor3f(0, 1, 0);
		if (simulation->points1.size() > 0)
			glVertex3f(simulation->points1[0].first.x(), simulation->points1[0].first.y(), simulation->points1[0].first.z());
		for (int i = 0; i < simulation->points1.size(); i++)
		{
			glVertex3f(simulation->points1[i].first.x(), simulation->points1[i].first.y(), simulation->points1[i].first.z());
			glColor3f(simulation->points1[i].second[0], simulation->points1[i].second[1], simulation->points1[i].second[2]);
			glVertex3f(simulation->points1[i].first.x(), simulation->points1[i].first.y(), simulation->points1[i].first.z());
		}
		glEnd();
	}

	if (simulation->draw2)
	{
		glBegin(GL_LINES);
		glColor3f(0, 1, 0);
		if (simulation->points2.size() > 0)
			glVertex3f(simulation->points2[0].first.x(), simulation->points2[0].first.y(), simulation->points2[0].first.z());
		for (int i = 0; i < simulation->points2.size(); i++)
		{
			glVertex3f(simulation->points2[i].first.x(), simulation->points2[i].first.y(), simulation->points2[i].first.z());
			glColor3f(simulation->points2[i].second[0], simulation->points2[i].second[1], simulation->points2[i].second[2]);
			glVertex3f(simulation->points2[i].first.x(), simulation->points2[i].first.y(), simulation->points2[i].first.z());
		}

		glEnd();
	}

	
	glEnable(GL_LIGHTING);


	glPopMatrix();
	glutSwapBuffers();
}

void init(void)
{
	//glClearColor(3.0 / 255.0, 169.0 / 255.0, 244.0 / 255.0, 0.0);
	glClearColor(0, 0, 0, 0);
	// Enable Z-buffering, backface culling, and lighting
	glEnable(GL_DEPTH_TEST);
	glMatrixMode(GL_PROJECTION);
	glShadeModel(GL_SMOOTH);
	glLoadIdentity();
	gluPerspective(60, 1, 1, 1000);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	// Set eye point and lookat point
	gluLookAt(0, 225, 300, 0, 0, 0, 0, 1, 0);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);

	glLightfv(GL_LIGHT0, GL_AMBIENT, qaAmbientLight);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, qaDiffuseLight);
	glLightfv(GL_LIGHT0, GL_SPECULAR, qaSpecularLight);

	GLfloat qaAmbientLight1[] = { 0.2, 0.2, 0.2, 1.0 };
	GLfloat qaDiffuseLight1[] = { 1, 1.0, 1.0, 1.0 };
	GLfloat qaSpecularLight1[] = { .7, .7, .7, 1.0 };

	glLightfv(GL_LIGHT1, GL_AMBIENT, qaAmbientLight1);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, qaDiffuseLight1);
	glLightfv(GL_LIGHT1, GL_SPECULAR, qaSpecularLight1);

	GLfloat qaLightPosition[] = { 100, 100, 100 };
	GLfloat qaLightPosition1[] = { -100, 100, -100 };
	//glLightfv(GL_LIGHT0, GL_POSITION, qaLightPosition);
	//glLightfv(GL_LIGHT1, GL_POSITION, qaLightPosition1);
}

void reshapeFunc(GLint newWidth, GLint newHeight)
{
	if (newWidth > newHeight) // Keep a square viewport
		glViewport((newWidth - newHeight) / 2, 0, newHeight, newHeight);
	else
		glViewport(0, (newHeight - newWidth) / 2, newWidth, newWidth);
	init();
	glutPostRedisplay();
}

void rotateview(void)
{
	if (rotateon) {
		spin += xchange / 250.0;
		if (spin >= 360.0) spin -= 360.0;
		if (spin < 0.0) spin += 360.0;
		spinup -= ychange / 250.0;
		if (spinup > 89.0) spinup = 89.0;
		if (spinup < -89.0) spinup = -89.0;
	}
	glutPostRedisplay();
}

void mouse(int button, int state, int x, int y)
{
	switch (button) {
	case GLUT_LEFT_BUTTON:
		if (state == GLUT_DOWN) {
			lastx = x;
			lasty = y;
			xchange = 0;
			ychange = 0;
			rotateon = 1;
		}
		else if (state == GLUT_UP) {
			xchange = 0;
			ychange = 0;
			rotateon = 0;
		}
		break;

	default:
		break;
	}
}

void motion(int x, int y)
{
	xchange = x - lastx;
	ychange = y - lasty;
}

void keyboard(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 's':
		std::thread(&Simulation::simulate, simulation).detach();
		break;
	case 'r':
		simulation->shouldReset = true;
		break;
	case 'a':
		simulation->airResistanceOn = !simulation->airResistanceOn;
		break;
	case '1':
		simulation->len1 += 10;
		break;
	case '2':
		simulation->len1 -= 10;
		break;
	case '3':
		simulation->len2 += 10;
		break;
	case '4':
		simulation->len2 -= 10;
		break;
	case '0':
		simulation->draw1 = !simulation->draw1;
		break;
	case '-':
		simulation->draw2 = !simulation->draw2;
		break;
	case 'p':
		simulation->paused = !simulation->paused;
		break;
	case 'o':
		simulation->showballs = !simulation->showballs;
	}
}


int main(int argc, char** argv)
{
	simulation = new Simulation(10000000);

	GLint SubMenu1, SubMenu2, SubMenu3, SubMenu4;

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_MULTISAMPLE | GLUT_STENCIL);
	glutInitWindowSize(1024, 768);
	glutInitWindowPosition(100, 100);
	glutCreateWindow("Double Pendulum");
	init();
	rotateon = 0;
	glutDisplayFunc(display);
	glutMouseFunc(mouse);
	glutMotionFunc(motion);
	glutKeyboardFunc(keyboard);
	glutIdleFunc(rotateview);
	glutReshapeFunc(reshapeFunc);

	GLfloat LineRange[2];
	glGetFloatv(GL_LINE_WIDTH_RANGE, LineRange);
	std::cout << "Minimum Line Width " << LineRange[0] << " -- ";
	std::cout << "Maximum Line Width " << LineRange[1] << std::endl;


	glutMainLoop();
	return 0;
}