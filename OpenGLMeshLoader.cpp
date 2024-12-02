#include "TextureBuilder.h"
#include "Model_3DS.h"
#include "GLTexture.h"
#include <glut.h>
#include <math.h>

int WIDTH = 1920;
int HEIGHT = 1080;

GLuint tex;
char title[] = "3D Model Loader Sample";

// 3D Projection Options
GLdouble fovy = 90.0;
GLdouble aspectRatio = (GLdouble)WIDTH / (GLdouble)HEIGHT;
GLdouble zNear = 0.1;
GLdouble zFar = 100;

class Vector
{
public:
	GLdouble x, y, z;
	Vector() {}
	Vector(GLdouble _x, GLdouble _y, GLdouble _z) : x(_x), y(_y), z(_z) {}
	//================================================================================================//
	// Operator Overloading; In C++ you can override the behavior of operators for you class objects. //
	// Here we are overloading the += operator to add a given value to all vector coordinates.        //
	//================================================================================================//
	void operator +=(float value)
	{
		x += value;
		y += value;
		z += value;
	}
};

Vector Eye(5, 2, 5);
Vector At(5, 2, 5);
Vector Up(0, 1, 0);

int cameraZoom = 0;

// Model Variables
Model_3DS model_player;

// Textures
GLTexture tex_ground;

enum mode {
	FIRST_PERSON,
	THIRD_PERSON,
};

bool mouseEnabled = true;
float sensitivity = 0.2f;
float yaw = -90.0f;
float pitch = 0.0f;
int lastX = WIDTH / 2;
int lastY = HEIGHT / 2;
mode currentMode = FIRST_PERSON;

float radians(float degrees) {
	return degrees * 3.14159f / 180.0f;
}

//=======================================================================
// Lighting Configuration Function
//=======================================================================
//void InitLightSource()
//{
//	// Enable Lighting for this OpenGL Program
//	glEnable(GL_LIGHTING);
//
//	// Enable Light Source number 0
//	// OpengL has 8 light sources
//	glEnable(GL_LIGHT0);
//
//	// Define Light source 0 ambient light
//	GLfloat ambient[] = { 0.1f, 0.1f, 0.1, 1.0f };
//	glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
//
//	// Define Light source 0 diffuse light
//	GLfloat diffuse[] = { 0.5f, 0.5f, 0.5f, 1.0f };
//	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
//
//	// Define Light source 0 Specular light
//	GLfloat specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };
//	glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
//
//	// Finally, define light source 0 position in World Space
//	GLfloat light_position[] = { 0.0f, 10.0f, 0.0f, 1.0f };
//	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
//}

//=======================================================================
// Material Configuration Function
//======================================================================
//void InitMaterial()
//{
//	// Enable Material Tracking
//	glEnable(GL_COLOR_MATERIAL);
//
//	// Sich will be assigneet Material Properties whd by glColor
//	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
//
//	// Set Material's Specular Color
//	// Will be applied to all objects
//	GLfloat specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };
//	glMaterialfv(GL_FRONT, GL_SPECULAR, specular);
//
//	// Set Material's Shine value (0->128)
//	GLfloat shininess[] = { 96.0f };
//	glMaterialfv(GL_FRONT, GL_SHININESS, shininess);
//}

//=======================================================================
// OpengGL Configuration Function
//=======================================================================
void myInit(void)
{
	glClearColor(0.0, 0.0, 0.0, 0.0);

	glMatrixMode(GL_PROJECTION);

	glLoadIdentity();

	gluPerspective(fovy, aspectRatio, zNear, zFar);
	//*******************************************************************************************//
	// fovy:			Angle between the bottom and top of the projectors, in degrees.			 //
	// aspectRatio:		Ratio of width to height of the clipping plane.							 //
	// zNear and zFar:	Specify the front and back clipping planes distances from camera.		 //
	//*******************************************************************************************//

	glMatrixMode(GL_MODELVIEW);

	glLoadIdentity();

	gluLookAt(Eye.x, Eye.y, Eye.z, At.x, At.y, At.z, Up.x, Up.y, Up.z);
	//*******************************************************************************************//
	// EYE (ex, ey, ez): defines the location of the camera.									 //
	// AT (ax, ay, az):	 denotes the direction where the camera is aiming at.					 //
	// UP (ux, uy, uz):  denotes the upward orientation of the camera.							 //
	//*******************************************************************************************//

	//InitLightSource();

	//InitMaterial();

	glEnable(GL_DEPTH_TEST);

	glEnable(GL_NORMALIZE);
}

//=======================================================================
// Render Ground Function
//=======================================================================
void RenderGround()
{
	glDisable(GL_LIGHTING);	// Disable lighting 

	glColor3f(0.6, 0.6, 0.6);	// Dim the ground texture a bit

	glEnable(GL_TEXTURE_2D);	// Enable 2D texturing

	glBindTexture(GL_TEXTURE_2D, tex_ground.texture[0]);	// Bind the ground texture

	glPushMatrix();
	glBegin(GL_QUADS);
	glNormal3f(0, 1, 0);	// Set quad normal direction.
	glTexCoord2f(0, 0);		// Set tex coordinates ( Using (0,0) -> (5,5) with texture wrapping set to GL_REPEAT to simulate the ground repeated grass texture).
	glVertex3f(-20, 0, -20);
	glTexCoord2f(5, 0);
	glVertex3f(20, 0, -20);
	glTexCoord2f(5, 5);
	glVertex3f(20, 0, 20);
	glTexCoord2f(0, 5);
	glVertex3f(-20, 0, 20);
	glEnd();
	glPopMatrix();

	glEnable(GL_LIGHTING);	// Enable lighting again for other entites coming throung the pipeline.

	glColor3f(1, 1, 1);	// Set material back to white instead of grey used for the ground texture.
}

//=======================================================================
// Display Function
//=======================================================================
void myDisplay(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);



	//GLfloat lightIntensity[] = { 0.7, 0.7, 0.7, 1.0f };
	//GLfloat lightPosition[] = { 0.0f, 100.0f, 0.0f, 0.0f };
	//glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
	//glLightfv(GL_LIGHT0, GL_AMBIENT, lightIntensity);

	// Draw Ground
	RenderGround();
	glPushMatrix();
	glScalef(0.5, 0.5, 0.5);
	model_player.Draw();
	glPopMatrix();
	//// Draw Tree Model
	//glPushMatrix();
	//glTranslatef(10, 0, 0);
	//glScalef(0.7, 0.7, 0.7);
	//model_tree.Draw();
	//glPopMatrix();

	//// Draw house Model
	//glPushMatrix();
	//glRotatef(90.f, 1, 0, 0);
	//model_house.Draw();
	//glPopMatrix();

	//sky box
	//glPushMatrix();

	//GLUquadricObj* qobj;
	//qobj = gluNewQuadric();
	//glTranslated(50, 0, 0);
	//glRotated(90, 1, 0, 1);
	//glBindTexture(GL_TEXTURE_2D, tex);
	//gluQuadricTexture(qobj, true);
	//gluQuadricNormals(qobj, GL_SMOOTH);
	//gluSphere(qobj, 100, 100, 100);
	//gluDeleteQuadric(qobj);

	//glPopMatrix();

	glutSwapBuffers();
}

//=======================================================================
// Keyboard Function
//=======================================================================
void myKeyboard(unsigned char button, int x, int y)
{
	switch (button)
	{
	case 27: // ESC key
		mouseEnabled = !mouseEnabled;
		if (mouseEnabled) {
			glutSetCursor(GLUT_CURSOR_NONE);
			glutWarpPointer(WIDTH / 2, HEIGHT / 2);
		}
		else {
			glutSetCursor(GLUT_CURSOR_LEFT_ARROW);
		}
		break;
	default:
		break;
	}
	glutPostRedisplay();
}

//=======================================================================
// Motion Function
//=======================================================================
void myMotion(int x, int y)
{
	if (!mouseEnabled) return;

	float xoffset = (x - WIDTH / 2) * sensitivity;
	float yoffset = (HEIGHT / 2 - y) * sensitivity;

	// Reset mouse position to center
	glutWarpPointer(WIDTH / 2, HEIGHT / 2);

	// Update camera angles
	yaw += xoffset;
	pitch += yoffset;

	// Constrain pitch
	if (pitch > 89.0f) pitch = 89.0f;
	if (pitch < -89.0f) pitch = -89.0f;

	// Calculate new camera direction
	At.x = Eye.x + cos(radians(yaw)) * cos(radians(pitch));
	At.y = Eye.y + sin(radians(pitch));
	At.z = Eye.z + sin(radians(yaw)) * cos(radians(pitch));

	// Update the view
	glLoadIdentity();
	gluLookAt(Eye.x, Eye.y, Eye.z, At.x, At.y, At.z, Up.x, Up.y, Up.z);
	glutPostRedisplay();
}

//=======================================================================
// Mouse Function
//=======================================================================
void myMouse(int button, int state, int x, int y)
{
	
}

//=======================================================================
// Reshape Function
//=======================================================================
void myReshape(int w, int h)
{
	if (h == 0) {
		h = 1;
	}

	WIDTH = w;
	HEIGHT = h;

	// set the drawable region of the window
	glViewport(0, 0, w, h);

	// set up the projection matrix 
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(fovy, (GLdouble)WIDTH / (GLdouble)HEIGHT, zNear, zFar);

	// go back to modelview matrix so we can move the objects about
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(Eye.x, Eye.y, Eye.z, At.x, At.y, At.z, Up.x, Up.y, Up.z);
}

//=======================================================================
// Assets Loading Function
//=======================================================================
void LoadAssets()
{
	//// Loading Model files
	model_player.Load("Models/Scene1/Bench/bench.3ds");
	//model_house.Load("Models/house/house.3DS");
	//model_tree.Load("Models/tree/Tree1.3ds");

	//// Loading texture files
	tex_ground.Load("Textures/ground.bmp");
	//loadBMP(&tex, "Textures/blu-sky-3.bmp", true);
}

//=======================================================================
// Main Function
//=======================================================================
void main(int argc, char** argv)
{
	glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

	glutInitWindowSize(WIDTH, HEIGHT);

	glutInitWindowPosition(100, 150);

	glutCreateWindow(title);

	glutDisplayFunc(myDisplay);

	glutKeyboardFunc(myKeyboard);

	glutMouseFunc(myMouse);

	glutReshapeFunc(myReshape);

	glutPassiveMotionFunc(myMotion);

	myInit();
	LoadAssets();
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_NORMALIZE);
	glEnable(GL_COLOR_MATERIAL);

	glShadeModel(GL_SMOOTH);

	glutMainLoop();
}