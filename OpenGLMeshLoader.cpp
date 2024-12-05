#include "TextureBuilder.h"
#include "Model_3DS.h"
#include "GLTexture.h"
#include <btBulletDynamicsCommon.h>
#include <glut.h>
#include <math.h>
#include <lib3ds/file.h>
#include <lib3ds/mesh.h>
#include <unordered_map>
#include <vector>
#include <stdio.h>

int WIDTH = 1920;
int HEIGHT = 1080;
GLuint tex;
char title[] = "FPS Game";
GLdouble fovy = 90.0;
GLdouble aspectRatio = (GLdouble)WIDTH / (GLdouble)HEIGHT;
GLdouble zNear = 0.1;
GLdouble zFar = 250;
std::unordered_map<unsigned char, bool> keyState;
std::vector<Model_3DS> models;
btDiscreteDynamicsWorld* dynamicsWorld;
btRigidBody* groundRigidBody;
btDefaultCollisionConfiguration* collisionConfiguration;
btCollisionDispatcher* dispatcher;
btBroadphaseInterface* overlappingPairCache;
btSequentialImpulseConstraintSolver* solver;
btRigidBody* playerRigidBody = nullptr;
btTriangleMesh* mapTriangleMeshShape = nullptr;
btBvhTriangleMeshShape* mapCollisionShape = nullptr;
btScaledBvhTriangleMeshShape* scaledMeshShape = nullptr;
class Vector
{
public:
	GLdouble x, y, z;
	Vector() : x(0), y(0), z(0) {} // Initialize x, y, and z to 0
	Vector(GLdouble _x, GLdouble _y, GLdouble _z) : x(_x), y(_y), z(_z) {}
	void operator +=(float value)
	{
		x += value;
		y += value;
		z += value;
	}
};
Vector Eye(0, 15, 0);
Vector At(0, 5, 0);
Vector Up(0, 1, 0);
int cameraZoom = 0;

Model_3DS model_player;
Model_3DS model_crate1;
Model_3DS model_crate2;
Model_3DS model_crate3;
Model_3DS model_car;
Model_3DS model_enemy;
Model_3DS model_bench;
Model_3DS model_map2;
Model_3DS model_map1;
Model_3DS model_bench1;
Model_3DS model_boxes;
Model_3DS model_weapon;
Model_3DS model_munitions;
Model_3DS model_planks;
Model_3DS model_supplies;
Model_3DS model_target;
Model_3DS model_chair;
GLTexture tex_ground;

enum mode {
	FIRST_PERSON,
	THIRD_PERSON,
};

bool mouseEnabled = false;
float sensitivity = 0.1f;
float yaw = -90.0f;
float pitch = 0.0f;
int lastX = WIDTH / 2;
int lastY = HEIGHT / 2;
mode currentMode = FIRST_PERSON;

enum displayMode {
	MAIN_MENU,
	MAP_1,
	MAP_2,
};
displayMode currentDisplayMode = MAIN_MENU;

float radians(float degrees) {
	return degrees * 3.14159f / 180.0f;
}

void playerPhysics() {
	// Create player collision shape
	btCollisionShape* playerShape = new btCapsuleShape(0.5, 1.5); // radius, height

	// Create player motion state (initial position)
	btDefaultMotionState* playerMotionState =
		new btDefaultMotionState(btTransform(
			btQuaternion(0, 0, 0, 1),
			btVector3(0, 1, 0) // slightly above the ground
		));

	// Define player mass and inertia
	btScalar mass = 70.0f;
	btVector3 playerInertia(0, 0, 0);
	playerShape->calculateLocalInertia(mass, playerInertia);

	// Create player rigid body construction info
	btRigidBody::btRigidBodyConstructionInfo
		playerRigidBodyCI(mass, playerMotionState, playerShape, playerInertia);

	// Adjust some properties to make player movement more realistic
	playerRigidBodyCI.m_restitution = 0.1f; // Slight bounciness
	playerRigidBodyCI.m_friction = 2.0f; // Ground friction

	// Create player rigid body
	playerRigidBody = new btRigidBody(playerRigidBodyCI);

	// Disable rotation to keep player upright
	playerRigidBody->setAngularFactor(btVector3(0, 0, 0));

	// Add player to the dynamics world
	dynamicsWorld->addRigidBody(playerRigidBody);
}

void addStaticBody(const Model_3DS& model, const btVector3& position, const btVector3& scale) {
	// Calculate half-extents from the model's bounding box
	btVector3 halfExtents(
		(model.maxX - model.minX) * scale.x() / 2.0f,
		(model.maxY - model.minY) * scale.y() / 2.0f,
		(model.maxZ - model.minZ) * scale.z() / 2.0f
	);

	// Create collision shape
	btCollisionShape* shape = new btBoxShape(halfExtents);

	// Create motion state (position of the object)
	btDefaultMotionState* motionState = new btDefaultMotionState(
		btTransform(btQuaternion(0, 0, 0, 1), position)
	);

	// Create rigid body construction info (mass = 0 for static bodies)
	btRigidBody::btRigidBodyConstructionInfo rigidBodyCI(
		0, motionState, shape, btVector3(0, 0, 0)
	);

	// Create rigid body
	btRigidBody* rigidBody = new btRigidBody(rigidBodyCI);

	// Add to dynamics world
	dynamicsWorld->addRigidBody(rigidBody);
}

void addStaticBodyTriangleMesh(Model_3DS& model, const btVector3& position, const btVector3& scale) {
	// Create triangle mesh
	btTriangleMesh* triangleMesh = model.CreateBulletTriangleMesh();
	if (!triangleMesh) {
		printf("Failed to create triangle mesh\n");
		return;
	}

	// Create triangle mesh shape with scaling
	btBvhTriangleMeshShape* meshShape = new btBvhTriangleMeshShape(triangleMesh, true);
	btScaledBvhTriangleMeshShape* scaledMeshShape = new btScaledBvhTriangleMeshShape(meshShape, scale);

	// Create motion state (position of the object)
	btDefaultMotionState* motionState = new btDefaultMotionState(
		btTransform(btQuaternion(0, 0, 0, 1), position)
	);

	// Create rigid body construction info (mass = 0 for static bodies)
	btRigidBody::btRigidBodyConstructionInfo rigidBodyCI(
		0, motionState, scaledMeshShape, btVector3(0, 0, 0)
	);

	// Create rigid body
	btRigidBody* rigidBody = new btRigidBody(rigidBodyCI);

	// Add to dynamics world
	dynamicsWorld->addRigidBody(rigidBody);
}

void initPhysicsWorld() {
	// Create the collision configuration
	collisionConfiguration = new btDefaultCollisionConfiguration();

	// Create the dispatcher
	dispatcher = new btCollisionDispatcher(collisionConfiguration);

	// Create the broadphase interface
	overlappingPairCache = new btDbvtBroadphase();

	// Create the constraint solver
	solver = new btSequentialImpulseConstraintSolver;

	// Create the dynamics world
	dynamicsWorld = new btDiscreteDynamicsWorld(
		dispatcher,
		overlappingPairCache,
		solver,
		collisionConfiguration
	);

	// Set gravity (default is -9.8 m/s^2 on Y-axis)
	dynamicsWorld->setGravity(btVector3(0, -9.8f * 6, 0));

	// Create ground plane
	mapTriangleMeshShape = model_map2.CreateBulletTriangleMesh();
	if (!mapTriangleMeshShape) {
		printf("Failed to create map triangle mesh shape\n");
		return;
	}

	mapCollisionShape = new btBvhTriangleMeshShape(mapTriangleMeshShape, true);
	if (!mapCollisionShape) {
		printf("Failed to create map collision shape\n");
		return;
	}

	// Create collision shape with scaling
	btVector3 meshScale(3.0f, 3.0f, 3.0f);
	scaledMeshShape = new btScaledBvhTriangleMeshShape(mapCollisionShape, meshScale);

	// Set up the rigid body with translation
	btDefaultMotionState* motionState = new btDefaultMotionState(
		btTransform(
			btQuaternion(0, 0, 0, 1),
			btVector3(5, -1, 5)  // Translation vector
		)
	);

	btRigidBody::btRigidBodyConstructionInfo rigidBodyCI(
		0,                        // Mass = 0 for static objects
		motionState,
		scaledMeshShape,
		btVector3(0, 0, 0)        // No local inertia for static objects
	);

	btRigidBody* rigidBody = new btRigidBody(rigidBodyCI);
	dynamicsWorld->addRigidBody(rigidBody);

	addStaticBody(model_crate1, btVector3(25, 0, 0), btVector3(0.5, 0.5, 0.5));   // model_crate1
	addStaticBodyTriangleMesh(model_crate2, btVector3(5, 0, -10), btVector3(0.075, 0.075, 0.075)); // model_crate2
	addStaticBodyTriangleMesh(model_crate2, btVector3(15, 0, -10), btVector3(0.075, 0.075, 0.075)); // model_crate2
	addStaticBodyTriangleMesh(model_crate3, btVector3(5, 0, 5), btVector3(0.5, 0.5, 0.5));    // model_crate3
	addStaticBodyTriangleMesh(model_crate3, btVector3(15, 0, 5), btVector3(0.5, 0.5, 0.5));   // model_crate3
	addStaticBody(model_car, btVector3(-3, 0, -5), btVector3(7, 7, 7));        // model_car
	addStaticBody(model_bench, btVector3(8, 0, 17), btVector3(0.01, 0.01, 0.01));// model_bench
	//addStaticBody(model_bench1, btVector3(0, 3, 15), btVector3(0.075, 0.075, 0.075)); // model_bench1
	//addStaticBody(model_boxes, btVector3(5, 2, 5), btVector3(0.1, 0.1, 0.1));    // model_boxes
	//addStaticBody(model_weapon, btVector3(0, 0, 0), btVector3(0.1, 0.1, 0.1));    // model_weapon
	//addStaticBody(model_munitions, btVector3(0, 0, 0), btVector3(0.1, 0.1, 0.1));    // model_munitions
	//addStaticBody(model_planks, btVector3(0, 0, -5), btVector3(0.5, 0.5, 0.5));   // model_planks
	//addStaticBody(model_supplies, btVector3(-5, 0, 5), btVector3(0.1, 0.1, 0.1));   // model_supplies
	//addStaticBody(model_target, btVector3(0, 0, -10), btVector3(0.025, 0.025, 0.025)); // model_target
	//addStaticBody(model_target, btVector3(-5, 0, -10), btVector3(0.025, 0.025, 0.025)); // model_target
	//addStaticBody(model_target, btVector3(5, 0, -10), btVector3(0.025, 0.025, 0.025));  // model_target
	//addStaticBody(model_chair, btVector3(5, 2, 15), btVector3(0.025, 0.025, 0.025));   // model_chair

	playerPhysics();
}

void cleanupPhysicsWorld() {
	// Remove and delete player rigid body
	if (playerRigidBody) {
		dynamicsWorld->removeRigidBody(playerRigidBody);
		delete playerRigidBody->getMotionState();
		delete playerRigidBody;
		playerRigidBody = nullptr;
	}

	// Remove ground body from dynamics world
	dynamicsWorld->removeRigidBody(groundRigidBody);

	// Delete physics objects
	delete groundRigidBody->getMotionState();
	delete groundRigidBody;

	// Delete world and solver components
	delete dynamicsWorld;
	delete solver;
	delete overlappingPairCache;
	delete dispatcher;
	delete collisionConfiguration;
}

void updatePhysics(float deltaTime) {
	dynamicsWorld->stepSimulation(deltaTime, 10);
}

void LoadAssets()
{
	model_player.Load("Models/Scene2/Player/player2.3ds");

	//// Loading Map2 files
	model_car.Load("Models/Scene2/BrokenCar/car.3ds");
	model_crate1.Load("Models/Scene2/crate1/crate.3ds");
	model_crate2.Load("Models/Scene2/crate2/crate.3ds");
	model_crate3.Load("Models/Scene2/crate3/crate.3ds");
	model_enemy.Load("Models/Scene2/Enemy/enemy.3ds");
	model_bench.Load("Models/Scene2/Bench/bench.3ds");
	model_map2.Load("Models/Scene2/Map/map3.3ds");

	//// Loading Map1 files
	model_map1.Load("Models/Scene1/Map/map.3ds");
	model_bench1.Load("Models/Scene1/Bench/bench.3ds");
	model_boxes.Load("Models/Scene1/Boxes/boxes.3ds");
	model_weapon.Load("Models/Scene1/Weapon/weapon.3ds");
	model_munitions.Load("Models/Scene1/Munitions/munitions.3ds");
	model_planks.Load("Models/Scene1/Planks/plank.3ds");
	model_supplies.Load("Models/Scene1/Supplies/Supplies.3DS");
	model_target.Load("Models/Scene1/Target/target.3ds");
	model_chair.Load("Models/Scene1/Chair/chair.3ds");

	models.push_back(model_player);
	models.push_back(model_crate1);
	models.push_back(model_crate2);
	models.push_back(model_crate3);
	models.push_back(model_car);
	models.push_back(model_enemy);
	models.push_back(model_bench);
	models.push_back(model_bench1);
	models.push_back(model_boxes);
	models.push_back(model_weapon);
	models.push_back(model_munitions);
	models.push_back(model_planks);
	models.push_back(model_supplies);
	models.push_back(model_target);
	models.push_back(model_chair);

	//// Loading texture files
	tex_ground.Load("Textures/ground.bmp");
	loadBMP(&tex, "Textures/blu-sky-3.bmp", true);
}

void initMaterials() {
	// Set consistent material properties
	float MatAmbient[] = { 0.2f, 0.2f, 0.2f, 1.0f };     // Some ambient reflection
	float MatDiffuse[] = { 0.8f, 0.8f, 0.8f, 1.0f };     // Consistent diffuse reflection
	float MatSpecular[] = { 0.5f, 0.5f, 0.5f, 1.0f };    // Some specular highlights
	float MatShininess = 50.0f;                          // Moderate shininess

	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, MatAmbient);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, MatDiffuse);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, MatSpecular);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, MatShininess);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
}

void setupLights() {
	// Position light relative to camera/view
	GLfloat lightPosition[] = { 0.0f, 0.0f, 10.0f, 1.0f };  // Light positioned in front of camera
	glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);

	// Set light properties
	GLfloat lightAmbient[] = { 0.2f, 0.2f, 0.2f, 1.0f };
	GLfloat lightDiffuse[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	GLfloat lightSpecular[] = { 1.0f, 1.0f, 1.0f, 1.0f };

	glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpecular);
}

void myInit(void)
{
	glClearColor(0.0, 0.0, 0.0, 0.0);

	glMatrixMode(GL_PROJECTION);

	glLoadIdentity();

	gluPerspective(fovy, aspectRatio, zNear, zFar);

	glMatrixMode(GL_MODELVIEW);

	glLoadIdentity();

	gluLookAt(Eye.x, Eye.y, Eye.z, At.x, At.y, At.z, Up.x, Up.y, Up.z);

	glEnable(GL_DEPTH_TEST);

	glEnable(GL_NORMALIZE);

	initMaterials();

	LoadAssets();

	initPhysicsWorld(); // Initialize Bullet Physics
}



void renderMap2() {
	//SIDE 1

	// Draw Crate Model
	glPushMatrix();
	glTranslatef(25, 0, 0);
	glScalef(0.5, 0.5, 0.5);
	model_crate1.Draw();
	glPopMatrix();

	// Draw Crate Model
	glPushMatrix();
	glTranslatef(5, 0, -10);
	glRotatef(90, 1, 0, 0);
	glScalef(0.075, 0.075, 0.075);
	model_crate2.Draw();
	glPopMatrix();

	// Draw Crate Model
	glPushMatrix();
	glTranslatef(15, 0, -10);
	glRotatef(90, 1, 0, 0);
	glScalef(0.075, 0.075, 0.075);
	model_crate2.Draw();
	glPopMatrix();

	//SIDE 2

	// Draw Crate Model
	glPushMatrix();
	glTranslatef(5, 0, 5);
	glRotatef(-90, 1, 0, 0);
	glScalef(0.5, 0.5, 0.5);
	glPushMatrix();
	glRotatef(90, 0, 0, 1);
	model_crate3.Draw();
	glPopMatrix();
	glPopMatrix();

	// Draw Crate Model
	glPushMatrix();
	glTranslatef(15, 0, 5);
	glRotatef(-90, 1, 0, 0);
	glScalef(0.5, 0.5, 0.5);
	glPushMatrix();
	glRotatef(90, 0, 0, 1);
	model_crate3.Draw();
	glPopMatrix();
	glPopMatrix();

	// Draw Car Model
	glPushMatrix();
	glTranslatef(-3, 0, -5);
	glScalef(7, 7, 7);
	model_car.Draw();
	glPopMatrix();

	// Draw Bench Model
	glPushMatrix();
	glTranslatef(8, 0, 17);
	glRotatef(180, 0, 1, 0);
	glScalef(0.01, 0.01, 0.01);
	model_bench.Draw();
	glPopMatrix();

	// Draw Map Model
	glPushMatrix();
	glTranslatef(5, 0, 5);
	glScalef(3.0, 3.0, 3.0);
	model_map2.Draw();
	glPopMatrix();

}

void renderMap1() {
	// Draw Map Model
	glPushMatrix();
	glTranslatef(0, 0, 0);
	glScalef(3.0, 3.0, 3.0);
	model_map1.Draw();
	glPopMatrix();

	// Draw Bench Model
	glPushMatrix();
	glTranslatef(0, 3, 15);
	glRotatef(90, 0, 1, 0);
	glScalef(0.075, 0.075, 0.075);
	model_bench1.Draw();
	glPopMatrix();

	// Draw Boxes Model
	glPushMatrix();
	glTranslatef(5, 2, 5);
	glScalef(0.1, 0.1, 0.1);
	model_boxes.Draw();
	glPopMatrix();

	// Draw Weapon Model
	glPushMatrix();
	glScalef(0.1, 0.1, 0.1);
	model_weapon.Draw();
	glPopMatrix();

	// Draw Munitions Model
	glPushMatrix();
	glScalef(0.1, 0.1, 0.1);
	model_munitions.Draw();
	glPopMatrix();

	// Draw Planks Model
	glPushMatrix();
	glTranslatef(0, 0, -5);
	glRotatef(-90, 1, 0, 0);
	glScalef(0.5, 0.5, 0.5);
	model_planks.Draw();
	glPopMatrix();

	// Draw Supplies Model
	glPushMatrix();
	glTranslatef(-5, 0, 5);
	glRotatef(90, 0, 1, 0);
	glScalef(0.1, 0.1, 0.1);
	model_supplies.Draw();
	glPopMatrix();

	// Draw Target Model
	glPushMatrix();
	glTranslatef(0, 0, -10);
	glScalef(0.025, 0.025, 0.025);
	model_target.Draw();
	glPopMatrix();

	//// Draw Target Model
	glPushMatrix();
	glTranslatef(-5, 0, -10);
	glScalef(0.025, 0.025, 0.025);
	model_target.Draw();
	glPopMatrix();

	//// Draw Target Model
	glPushMatrix();
	glTranslatef(5, 0, -10);
	glScalef(0.025, 0.025, 0.025);
	model_target.Draw();
	glPopMatrix();

	// Draw Chair Model
	glPushMatrix();
	glTranslatef(5, 2, 15);
	glRotatef(45, 0, 1, 0);
	glScalef(0.025, 0.025, 0.025);
	model_chair.Draw();
	glPopMatrix();


}

void drawPlayer() {
	// Update player position from physics simulation
	if (playerRigidBody) {
		btTransform trans;
		playerRigidBody->getMotionState()->getWorldTransform(trans);

		// Update Eye position based on player rigid body
		Eye.x = trans.getOrigin().x();
		Eye.y = trans.getOrigin().y() + 3; // Adjust height as needed
		Eye.z = trans.getOrigin().z();

		// Update At position to maintain camera direction
		At.x = Eye.x + cos(radians(yaw)) * cos(radians(pitch));
		At.y = Eye.y + sin(radians(pitch));
		At.z = Eye.z + sin(radians(yaw)) * cos(radians(pitch));
	}
}

void displayText(float x, float y, int r, int g, int b, const char* string) {
	int j = strlen(string);

	glDisable(GL_DEPTH_TEST);
	glColor3f(r, g, b);
	glRasterPos2f(x, y);
	for (int i = 0; i < j; i++) {
		glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, string[i]);
	}
	glEnable(GL_DEPTH_TEST);
	glColor3f(1, 1, 1);
}

void drawButton(float x, float y, float width, float height, const char* text, void (*callback)()) {
    // Draw button background
    glColor3f(0.7f, 0.7f, 0.7f); // Light gray color for button
    glBegin(GL_QUADS);
    glVertex2f(x - width/2, y - height/2);      // Bottom left
    glVertex2f(x + width/2, y - height/2);      // Bottom right
    glVertex2f(x + width/2, y + height/2);      // Top right
    glVertex2f(x - width/2, y + height/2);      // Top left
    glEnd();
	glColor3f(1, 1, 1);

    // Draw button border
    glColor3f(0.4f, 0.4f, 0.4f); // Darker gray for border
    glLineWidth(2.0f);
    glBegin(GL_LINE_LOOP);
    glVertex2f(x - width/2, y - height/2);
    glVertex2f(x + width/2, y - height/2);
    glVertex2f(x + width/2, y + height/2);
    glVertex2f(x - width/2, y + height/2);
    glEnd();
	glColor3f(1, 1, 1);

	glDisable(GL_DEPTH_TEST);
    // Draw button text
    glColor3f(0.0f, 0.0f, 0.0f); // Black text
    float textX = x - (glutBitmapLength(GLUT_BITMAP_TIMES_ROMAN_24, (const unsigned char*)text) / 2.0f);
    float textY = y - 10; // Adjust vertical position to center text
    glRasterPos2f(textX, textY);
    
    // Display text character by character
    for (int i = 0; text[i] != '\0'; i++) {
        glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, text[i]);
    }
	glColor3f(1, 1, 1);
	glEnable(GL_DEPTH_TEST);
}

void drawMainMenu() {
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glClearColor(0.5, 0.5, 0.5, 1.0);
	glLoadIdentity();
	gluOrtho2D(-0.5 * WIDTH, 0.5 * WIDTH, -0.5 * HEIGHT, 0.5 * HEIGHT); // Set up an orthographic projection

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	displayText(-100, 150, 1, 1, 1, "Pick a game mode");

	// button for map 1
	drawButton(0, 70, 200, 50, "Map 1", []() {
		currentDisplayMode = MAP_1;
	});

	// button for map 2
	drawButton(0, -70, 200, 50, "Map 2", []() {
		currentDisplayMode = MAP_2;
	});

	glPopMatrix();

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
}

void myDisplay(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	setupLights();

	// if current display mode is MAIN_MENU, show gray screen with text
	if(currentDisplayMode == MAIN_MENU) {
		drawMainMenu();
		glutSwapBuffers();
		return;
	}
	else if(currentDisplayMode == MAP_1) {
		// Draw Player Model
		glPushMatrix();
		glScalef(2, 2, 2);
		glTranslatef(0, 0, 2.5);
		glRotatef(90, 1, 0, 0);
		model_player.Draw();
		glPopMatrix();
		drawPlayer();

		// Draw Map Model
		renderMap1();

		//sky box
		glPushMatrix();

		GLUquadricObj* qobj;
		qobj = gluNewQuadric();
		glTranslated(50, 0, 0);
		glRotated(90, 1, 0, 1);
		glBindTexture(GL_TEXTURE_2D, tex);
		gluQuadricTexture(qobj, true);
		gluQuadricNormals(qobj, GL_SMOOTH);
		gluSphere(qobj, 200, 100, 100);
		gluDeleteQuadric(qobj);

		glPopMatrix();

		// Optional: Add physics debug drawing
		dynamicsWorld->debugDrawWorld();


		glutSwapBuffers();
	}
	else if(currentDisplayMode == MAP_2) {
	
		// Draw Player Model
		glPushMatrix();
		glScalef(2, 2, 2);
		glTranslatef(0, 0, 2.5);
		glRotatef(90, 1, 0, 0);
		model_player.Draw();
		glPopMatrix();
		drawPlayer();

		//// Draw Enemy Model
		//glPushMatrix();
		//glTranslatef(0, 0, 10);
		//glScalef(0.5, 0.5, 0.5);
		//glRotatef(-90, 0, 1, 0);
		//model_enemy.Draw();
		//glPopMatrix();


		// Draw Map Model
		renderMap2();

		//sky box
		glPushMatrix();

		GLUquadricObj* qobj;
		qobj = gluNewQuadric();
		glTranslated(50, 0, 0);
		glRotated(90, 1, 0, 1);
		glBindTexture(GL_TEXTURE_2D, tex);
		gluQuadricTexture(qobj, true);
		gluQuadricNormals(qobj, GL_SMOOTH);
		gluSphere(qobj, 200, 100, 100);
		gluDeleteQuadric(qobj);

		glPopMatrix();

		// Optional: Add physics debug drawing
		dynamicsWorld->debugDrawWorld();
		glutSwapBuffers();
	}
}

void myKeyboard(unsigned char button, int x, int y)
{
	keyState[button] = true; // Mark key as pressed

	if (button == 27) // ESC key to toggle mouse
	{
		mouseEnabled = !mouseEnabled;
		if (mouseEnabled) {
			glutSetCursor(GLUT_CURSOR_NONE);
			glutWarpPointer(WIDTH / 2, HEIGHT / 2);
		}
		else {
			glutSetCursor(GLUT_CURSOR_LEFT_ARROW);
		}
	}
	else if (button == 'f') // Resize and reposition window
	{
		glutReshapeWindow(800, 600);
		glutPositionWindow(100, 100);
	}
	if (button == ' ') {
		if (abs(playerRigidBody->getLinearVelocity().y()) < 0.1f) {
			playerRigidBody->applyCentralImpulse(btVector3(0, 1000, 0)); // Jump impulse
		}
	}
	glutPostRedisplay();
	//if (!playerRigidBody) return;

	//float moveSpeed = 60.0f;
	//btVector3 moveDirection(0, 0, 0);

	//switch (button)
	//{
	//case 'w': // Move forward
	//	moveDirection.setX(moveSpeed);
	//	break;
	//case 's': // Move backward
	//	moveDirection.setX(-moveSpeed);
	//	break;
	//case 'a': // Move left
	//	moveDirection.setZ(moveSpeed);
	//	break;
	//case 'd': // Move right
	//	moveDirection.setZ(-moveSpeed);
	//	break;
	//case ' ': // Jump
	//	// Only allow jumping if player is somewhat grounded
	//	if (abs(playerRigidBody->getLinearVelocity().y()) < 0.1f) {
	//		playerRigidBody->applyCentralImpulse(btVector3(0, 600, 0)); // Jump impulse
	//	}
	//	break;
	//case 27: // ESC key
	//	mouseEnabled = !mouseEnabled;
	//	if (mouseEnabled) {
	//		glutSetCursor(GLUT_CURSOR_NONE);
	//		glutWarpPointer(WIDTH / 2, HEIGHT / 2);
	//	}
	//	else {
	//		glutSetCursor(GLUT_CURSOR_LEFT_ARROW);
	//	}
	//	break;
	//case 'f':
	//	glutReshapeWindow(800, 600);
	//	glutPositionWindow(100, 100);
	//	break;
	//}

	//// Convert move direction to world space considering camera orientation
	//btVector3 forward(sin(radians(yaw)), 0, -cos(radians(yaw)));
	//btVector3 right(forward.cross(btVector3(0, 1, 0)));

	//btVector3 worldMoveDirection =
	//	forward * moveDirection.z() +
	//	right * moveDirection.x();

	//// Apply movement
	//playerRigidBody->activate(true); // Wake up the rigid body
	//playerRigidBody->applyCentralImpulse(worldMoveDirection);
	//glutPostRedisplay();
}

void myKeyboardUp(unsigned char button, int x, int y)
{
	keyState[button] = false; // Mark key as released
}

void updateMovement() {
	if (!playerRigidBody) return;

	float moveSpeed = 10.0f; // Target movement speed
	float airControlFactor = 0.2f; // Movement control in air
	btVector3 moveDirection(0, 0, 0);

	// Check key states
	if (keyState['w']) moveDirection += btVector3(1, 0, 0); // Forward
	if (keyState['s']) moveDirection += btVector3(-1, 0, 0);  // Backward
	if (keyState['a']) moveDirection += btVector3(0, 0, 1); // Left
	if (keyState['d']) moveDirection += btVector3(0, 0, -1);  // Right

	// Normalize the movement direction
	if (moveDirection.length() > 0) {
		moveDirection.normalize();
	}

	// Convert movement direction to world space
	btVector3 forward(sin(radians(yaw)), 0, -cos(radians(yaw)));
	btVector3 right(forward.cross(btVector3(0, 1, 0)));
	btVector3 worldMoveDirection =
		forward * moveDirection.z() +
		right * moveDirection.x();

		// Set velocity directly for ground movement
	btVector3 newVelocity = worldMoveDirection * moveSpeed;
	newVelocity.setY(playerRigidBody->getLinearVelocity().y()); // Preserve vertical velocity
	playerRigidBody->setLinearVelocity(newVelocity);
}

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

void myMouse(int button, int state, int x, int y)
{
	// Convert screen coordinates to OpenGL coordinates
	float glX = (x - WIDTH / 2.0f) * (WIDTH / (float)WIDTH);
	float glY = (HEIGHT / 2.0f - y) * (HEIGHT / (float)HEIGHT);

	// Check if mouse click is within button bounds
	// Example for Map 1 button
	if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
		// Button coordinates match those in drawMainMenu
		if (glX >= -100 && glX <= 100 && glY >= 45 && glY <= 95) {
			printf("Map 1 clicked\n");
			currentDisplayMode = MAP_1;
			mouseEnabled = true;
		}

		// Similar check for Map 2 button
		if (glX >= -100 && glX <= 100 && glY >= -95 && glY <= -45) {
			printf("Map 2 clicked\n");
			currentDisplayMode = MAP_2;
			mouseEnabled = true;
		}
	}

	glutPostRedisplay();
}

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

void onUpdate(int value) {
    // Update physics simulation
    updatePhysics(1.0f / 144.0f); // Assuming 144 FPS

    updateMovement();

    // Request a redraw
    glutPostRedisplay();

    // Set up the next timer event
    glutTimerFunc(7, onUpdate, 0);  // ~144 FPS
}


void main(int argc, char** argv)
{
	glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

	glutInitWindowSize(WIDTH, HEIGHT);

	glutInitWindowPosition(100, 150);

	glutCreateWindow(title);

	glutTimerFunc(0, onUpdate, 0);

	glutFullScreen();

	glutDisplayFunc(myDisplay);

	glutKeyboardFunc(myKeyboard);

	glutMouseFunc(myMouse);

	glutReshapeFunc(myReshape);

	glutPassiveMotionFunc(myMotion);

	glutKeyboardUpFunc(myKeyboardUp);


	myInit();
	
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_NORMALIZE);
	glEnable(GL_COLOR_MATERIAL);

	glShadeModel(GL_SMOOTH);

	glutMainLoop();

	cleanupPhysicsWorld(); // Cleanup physics world when done
}