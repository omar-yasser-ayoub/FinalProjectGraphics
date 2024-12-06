using namespace std;
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
std::unordered_map<int, bool> mouseState;
std::vector<Model_3DS> models;
btDiscreteDynamicsWorld* dynamicsWorld;
btRigidBody* mapRigidBody = nullptr;
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

	// Constructors
	Vector() : x(0), y(0), z(0) {} // Initialize x, y, and z to 0
	Vector(GLdouble _x, GLdouble _y, GLdouble _z) : x(_x), y(_y), z(_z) {}

	// Operator overloading for addition
	Vector operator+(const Vector& other) const
	{
		return Vector(x + other.x, y + other.y, z + other.z);
	}

	// Operator overloading for scalar multiplication
	Vector operator*(float scalar) const
	{
		return Vector(x * scalar, y * scalar, z * scalar);
	}

	Vector operator-(const Vector& other) const
	{
		return Vector(x - other.x, y - other.y, z - other.z);
	}

	Vector cross(const Vector& other) const
	{
		return Vector(
			y * other.z - z * other.y,
			z * other.x - x * other.z,
			x * other.y - y * other.x
		);
	}

	// Normalize the vector
	Vector normalize() const
	{
		GLdouble length = sqrt(x * x + y * y + z * z);
		if (length > 0)
			return Vector(x / length, y / length, z / length);
		return Vector();
	}

	btVector3 toBtVector3() const
	{
		return btVector3(static_cast<btScalar>(x), static_cast<btScalar>(y), static_cast<btScalar>(z));
	}
};
Vector Eye(0, 15, 0);
Vector At(0, 5, 0);
Vector Up(0, 1, 0);
int cameraZoom = 0;
int score = 0;

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
Model_3DS model_muzzle;
Model_3DS model_skybox;
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
mode currentMode = THIRD_PERSON;

enum displayMode {
	MAIN_MENU,
	MAP_1,
	MAP_2,
};
displayMode currentDisplayMode = MAIN_MENU;

int mouseEventX = 0;
int mouseEventY = 0;

float bobbingFactor = 0.0f;
float bobSpeed = 10.0f;
float bobAmount = 0.05f;
float swayAmount = 0.01f;
float oldTime = 0.0f;

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
	playerRigidBodyCI.m_friction = 0.2f; // Ground friction

	// Create player rigid body
	playerRigidBody = new btRigidBody(playerRigidBodyCI);

	// Disable rotation to keep player upright
	playerRigidBody->setAngularFactor(btVector3(0, 0, 0));

	// Add player to the dynamics world
	dynamicsWorld->addRigidBody(playerRigidBody);
}

void addStaticBody(const Model_3DS& model, const btVector3& position, const btVector3& scale, const std::string& name) {
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

	// Set the user pointer to the name (allocate a new string for the pointer)
	std::string* namePointer = new std::string(name);
	rigidBody->setUserPointer(static_cast<void*>(namePointer));

	// Add to dynamics world
	dynamicsWorld->addRigidBody(rigidBody);
}

void addStaticBodyTriangleMesh(Model_3DS& model, const btVector3& position, const btVector3& scale, const std::string& name) {
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

	std::string* namePointer = new std::string(name);
	rigidBody->setUserPointer(static_cast<void*>(namePointer));

	// Add to dynamics world
	dynamicsWorld->addRigidBody(rigidBody);
}

class BulletCollisionCallback : public btCollisionWorld::ContactResultCallback {
public:
	btCollisionObject* bulletObject;
	btCollisionObject* targetObject;
	bool collisionDetected;

	BulletCollisionCallback() : collisionDetected(false) {}

	virtual btScalar addSingleResult(btManifoldPoint& cp,
		const btCollisionObjectWrapper* colObj0, int partId0, int index0,
		const btCollisionObjectWrapper* colObj1, int partId1, int index1) override {

		// Check if the collision involves your bullet
		if ((colObj0->getCollisionObject() == bulletObject &&
			colObj1->getCollisionObject() == targetObject) ||
			(colObj1->getCollisionObject() == bulletObject &&
				colObj0->getCollisionObject() == targetObject)) {

			collisionDetected = true;
		}
		return 0.0f;
	}
};

void checkBulletCollision(btRigidBody* bullet, btRigidBody* enemy) {
	if (!bullet) {
		printf("BULLET IS NULL\n");
		return;
	}
	if (!enemy) {
		printf("ENEMY IS NULL\n");
		return;
	}
	if (!dynamicsWorld) {
		printf("DYNAMICS WORLD IS NULL\n");
		return;
	}

	try {
		BulletCollisionCallback callback;
		callback.bulletObject = bullet;
		callback.targetObject = enemy;

		// Additional null check before contact test
		if (bullet && enemy) {
			dynamicsWorld->contactPairTest(bullet, enemy, callback);

			if (callback.collisionDetected) {
				printf("Collision detected\n");
				// Handle collision logic safely
			}
		}
	}
	catch (std::exception& e) {
		printf("Collision check error: %s\n", e.what());
	}
}

void initPhysicsWorld(int map) {
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

	if (map == 1) {

		// Create ground plane
		mapTriangleMeshShape = model_map1.CreateBulletTriangleMesh();
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
				btVector3(0, -1, 0)  // Translation vector
			)
		);

		btRigidBody::btRigidBodyConstructionInfo rigidBodyCI(
			0,                        // Mass = 0 for static objects
			motionState,
			scaledMeshShape,
			btVector3(0, 0, 0)        // No local inertia for static objects
		);

		mapRigidBody = new btRigidBody(rigidBodyCI);
		dynamicsWorld->addRigidBody(mapRigidBody);

		//addStaticBody(model_bench1, btVector3(0, 3, 15), btVector3(0.075, 0.075, 0.075), "model_bench1"); // model_bench1
		//addStaticBody(model_boxes, btVector3(5, 1.5, 5), btVector3(0.1, 0.1, 0.1), "model_boxes"); // model_boxes
		//addStaticBody(model_munitions, btVector3(0, 0, 0), btVector3(0.1, 0.1, 0.1), "model_munitions"); // model_munitions
		//addStaticBody(model_planks, btVector3(0, 0, -5), btVector3(0.5, 0.5, 0.5), "model_planks"); // model_planks
		//addStaticBody(model_supplies, btVector3(-5, 0, 5), btVector3(0.1, 0.1, 0.1), "model_supplies"); // model_supplies
		addStaticBodyTriangleMesh(model_target, btVector3(0, 0, -10), btVector3(0.025, 0.025, 0.025), "model_target_1"); // model_target_1
		addStaticBodyTriangleMesh(model_target, btVector3(-5, 0, -10), btVector3(0.025, 0.025, 0.025), "model_target_2"); // model_target_2
		addStaticBodyTriangleMesh(model_target, btVector3(5, 0, -10), btVector3(0.025, 0.025, 0.025), "model_target_3"); // model_target_3
		//addStaticBody(model_chair, btVector3(5, 2, 15), btVector3(0.025, 0.025, 0.025), "model_chair"); // model_chair

		playerPhysics();

	}
	else if (map == 2) {

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

		mapRigidBody = new btRigidBody(rigidBodyCI);
		dynamicsWorld->addRigidBody(mapRigidBody);

		addStaticBody(model_crate1, btVector3(25, 0, 0), btVector3(0.5, 0.5, 0.5), "model_crate1"); // model_crate1
		addStaticBody(model_car, btVector3(-3, 0, -5), btVector3(7, 7, 7), "model_car"); // model_car
		addStaticBody(model_bench, btVector3(8, 0, 17), btVector3(0.01, 0.01, 0.01), "model_bench"); // model_bench
		playerPhysics();

	}

	
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
	dynamicsWorld->removeRigidBody(mapRigidBody);

	// Delete physics objects
	delete mapRigidBody->getMotionState();
	delete mapRigidBody;

	// Delete world and solver components
	delete dynamicsWorld;
	delete solver;
	delete overlappingPairCache;
	delete dispatcher;
	delete collisionConfiguration;
}

void updatePhysics(float deltaTime) {
	if (!dynamicsWorld) return;
	if (currentDisplayMode == MAIN_MENU) return;
	dynamicsWorld->stepSimulation(deltaTime, 10);
}

void LoadAssets()
{
	model_player.Load("Models/Scene2/Player/player2.3ds");
	model_muzzle.Load("Models/Scene2/Muzzle/muzzle2.3ds");
	model_skybox.Load("Models/Scene1/SkyBoxMap/skybox.3ds");

	//// Loading Map2 files
	model_car.Load("Models/Scene2/BrokenCar/car.3ds");
	model_crate1.Load("Models/Scene2/crate1/crate.3ds");
	model_enemy.Load("Models/Scene2/Enemy/enemy.3ds");
	model_bench.Load("Models/Scene2/Bench/bench.3ds");
	model_map2.Load("Models/Scene2/Map/map6.3ds");

	//// Loading Map1 files
	model_map1.Load("Models/Scene1/Map/map3.3ds");
	//model_bench1.Load("Models/Scene1/Bench/bench.3ds");
	//model_boxes.Load("Models/Scene1/Boxes/boxes.3ds");
	model_weapon.Load("Models/Scene1/Weapon/weapon.3ds");
	//model_munitions.Load("Models/Scene1/Munitions/munitions.3ds");
	//model_planks.Load("Models/Scene1/Planks/plank.3ds");
	//model_supplies.Load("Models/Scene1/Supplies/Supplies.3DS");
	model_target.Load("Models/Scene1/Target/target.3ds");
	//model_chair.Load("Models/Scene1/Chair/chair.3ds");

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

	oldTime = glutGet(GLUT_ELAPSED_TIME);

	initMaterials();

	LoadAssets();
}

void renderMap2() {
	//SIDE 1

	// Draw Crate Model
	glPushMatrix();
	glTranslatef(25, 0, 0);
	glScalef(0.5, 0.5, 0.5);
	model_crate1.Draw();
	glPopMatrix();

	//SIDE 2

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
}

void DrawCrosshair(int screenWidth, int screenHeight) {
	// Disable depth testing to ensure the crosshair is always visible
	glDisable(GL_DEPTH_TEST);

	// Set up orthographic projection for 2D rendering
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D(0, screenWidth, 0, screenHeight); // Screen-space coordinates
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	// Set crosshair color (e.g., white)
	glColor3f(0, 1.0f, 0);

	// Crosshair size and position
	float crosshairSize = 10.0f;  // Size of the crosshair
	float centerX = screenWidth / 2.0f;
	float centerY = screenHeight / 2.0f;

	// Draw the crosshair (two lines)
	glBegin(GL_LINES);
	// Horizontal line
	glVertex2f(centerX - crosshairSize, centerY);
	glVertex2f(centerX + crosshairSize, centerY);

	// Vertical line
	glVertex2f(centerX, centerY - crosshairSize);
	glVertex2f(centerX, centerY + crosshairSize);
	glEnd();

	glColor3f(1.0f, 1.0f, 1.0f);

	// Restore previous matrix modes
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

	// Re-enable depth testing
	glEnable(GL_DEPTH_TEST);
}

void displayScore() {
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D(-2.0, 2.0, -1.0, 1.0); // Set up an orthographic projection

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	glRasterPos2f(-1.9, 0.9); // Set the position of the text

	std::string s = "Score: " + std::to_string(score);
	for (int i = 0; i < s.length(); i++) {
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, s[i]);
	}
	glPopMatrix();

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
}

void drawPlayer() {
	// Update player position and rotation from physics simulation
	if (playerRigidBody) {
		btTransform trans;
		playerRigidBody->getMotionState()->getWorldTransform(trans);

		// Get current time to calculate frame delta for smooth motion
		float currentTime = glutGet(GLUT_ELAPSED_TIME);
		float deltaTime = (currentTime - oldTime) / 1000.0f; // Time in seconds
		oldTime = currentTime;

		if (currentMode == THIRD_PERSON) {
			glPushMatrix();
			// Translate to player's position
			glTranslatef(trans.getOrigin().x(), trans.getOrigin().y(), trans.getOrigin().z());
			// Apply additional scaling and fixed rotations
			glScalef(2, 2, 2);
			glRotatef(90, 1, 0, 0);
			glRotatef(180, 0, 0, 1);
			glRotatef(yaw + 90, 0, 0, 1); // Rotate around Y-axis (yaw)

			// Draw the player model
			model_player.Draw();

			glPopMatrix();
		}

		if (currentMode == FIRST_PERSON) {
			glPushMatrix();
			glLoadIdentity();

			// Apply weapon bobbing
			if (keyState['w'] || keyState['a'] || keyState['s'] || keyState['d']) {
				bobbingFactor += deltaTime * bobSpeed;
				if (bobbingFactor > 2 * 3.141592653) {
					bobbingFactor -= 2 * 3.141592653;
				}

				// Vertical bobbing (up and down movement)
				float verticalBobbing = sin(bobbingFactor) / 2 * bobAmount;

				// Horizontal sway (left and right movement)
				float horizontalSway = cos(bobbingFactor) / 2 * swayAmount;

				// Offset the weapon position based on bobbing
				float weaponOffsetX = 0.3f + horizontalSway; // Apply horizontal sway to X
				float weaponOffsetY = -0.4f + verticalBobbing; // Apply vertical bobbing to Y
				float weaponOffsetZ = -0.5f;

				// Apply weapon offset to the matrix
				glTranslatef(weaponOffsetX, weaponOffsetY, weaponOffsetZ);
			}
			else {
				// When the player is not moving, reset the bobbing offset
				glTranslatef(0.3f, -0.4f, -0.5f);
			}

			glScalef(0.01f, 0.01f, 0.01f);
			glRotatef(180, 0, 1, 0); // Rotate weapon to face the camera

			// Draw the weapon model
			model_weapon.Draw();

			// Draw the muzzle flash if the player is shooting
			if (mouseState[GLUT_LEFT_BUTTON]) {  // Assuming `isShooting` is true when the player fires
				glPushMatrix();
				// Translate muzzle flash in front of the weapon
				glTranslatef(2.0f, 10.0f, 50.0f); // Adjust this for proper position in front of the weapon

				glScalef(4.0f, 4.0f, 4.0f);
				// Draw a simple sphere (you can replace this with a model or particle system)
				model_muzzle.Draw();

				// Disable blending after drawing the muzzle flash
				glDisable(GL_BLEND);

				glPopMatrix();
			}

			glPopMatrix();
		}

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

void displayText(float x, float y, int r, int g, int b, const char* string, void* font) {
	int j = strlen(string);

	glDisable(GL_DEPTH_TEST);
	glColor3f(r, g, b);
	glRasterPos2f(x, y);
	for (int i = 0; i < j; i++) {
		glutBitmapCharacter(font, string[i]);
	}
	glEnable(GL_DEPTH_TEST);
	glColor3f(1, 1, 1);
}

void drawButton(float x, float y, float width, float height, const char* text, void (*callback)()) {
    // Draw button background
    glColor3f(1.0f, 0.85f, 0.58f); // Light gray color for button
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
    float textX = x - (glutBitmapLength(GLUT_BITMAP_TIMES_ROMAN_24, (const unsigned char*)text) / 2.0f);
    float textY = y - 10; // Adjust vertical position to center text
    glRasterPos2f(textX, textY);
    
    // Display text character by character
    for (int i = 0; text[i] != '\0'; i++) {
        glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, text[i]);
    }
	glEnable(GL_DEPTH_TEST);
}

void drawMainMenu() {
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glClearColor(0.89, 0.83, 0.67, 1.0);
	glLoadIdentity();
	gluOrtho2D(-0.5 * WIDTH, 0.5 * WIDTH, -0.5 * HEIGHT, 0.5 * HEIGHT);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	displayText(-60, 150, 1, 1, 1, "GUC Strike", GLUT_BITMAP_TIMES_ROMAN_24);
	displayText(-90, 50, 1, 1, 1, "Pick a game mode", GLUT_BITMAP_TIMES_ROMAN_24);

	// button for map 1
	drawButton(-150, -50, 200, 50, "Train", []() {
		currentDisplayMode = MAP_1;
	});

	// button for map 2
	drawButton(150, -50, 200, 50, "Play", []() {
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
		displayScore();
		// Draw Player Model
		glPushMatrix();
		drawPlayer();
		glPopMatrix();

		glEnable(GL_LIGHT1); // Sunlight
		glEnable(GL_LIGHT2); // Moonlight

		// Calculate skybox rotation
		float rotationAngle = oldTime / 1000.0f;

		// Configure sunlight (GL_LIGHT1)
		GLfloat sunPosition[] = { 0.0f, 1.0f, 0.0f, 1.0f };
		GLfloat sunDiffuse[] = { 1.0f, 0.8f, 0.5f, 1.0f };
		GLfloat sunSpecular[] = { 1.0f, 0.8f, 0.5f, 1.0f };

		glPushMatrix();
		glRotatef(rotationAngle, 1, 0, 1);
		glLightfv(GL_LIGHT1, GL_POSITION, sunPosition);
		glLightfv(GL_LIGHT1, GL_DIFFUSE, sunDiffuse);
		glLightfv(GL_LIGHT1, GL_SPECULAR, sunSpecular);
		glPopMatrix();

		// Configure moonlight (GL_LIGHT2)
		GLfloat moonPosition[] = { 0.0f, -1.0f, 0.0f, 1.0f };
		GLfloat moonDiffuse[] = { 0.3f, 0.5f, 1.0f, 1.0f };
		GLfloat moonSpecular[] = { 0.3f, 0.5f, 1.0f, 1.0f };

		glPushMatrix();
		glRotatef(rotationAngle, 1, 0, 1);
		glLightfv(GL_LIGHT2, GL_POSITION, moonPosition);
		glLightfv(GL_LIGHT2, GL_DIFFUSE, moonDiffuse);
		glLightfv(GL_LIGHT2, GL_SPECULAR, moonSpecular);
		glPopMatrix();

		// Draw Map Model
		renderMap1();

		//sky box
		glPushMatrix();

		glScalef(150, 150, 150);

		glRotatef(oldTime / 1000, 1, 0, 1);

		model_skybox.Draw();

		glPopMatrix();

		DrawCrosshair(WIDTH, HEIGHT);

		// Optional: Add physics debug drawing
		dynamicsWorld->debugDrawWorld();

		glutSwapBuffers();
	}
	else if(currentDisplayMode == MAP_2) {
		displayScore();
		// Draw Player Model
		glPushMatrix();
		drawPlayer();
		glPopMatrix();

		//// Draw Enemy Model
		//glPushMatrix();
		//glTranslatef(0, 0, 10);
		//glScalef(0.5, 0.5, 0.5);
		//glRotatef(-90, 0, 1, 0);
		//model_enemy.Draw();
		//glPopMatrix();

		glEnable(GL_LIGHT1); // Sunlight
		glEnable(GL_LIGHT2); // Moonlight

		// Calculate skybox rotation
		float rotationAngle = oldTime / 1000.0f;

		// Configure sunlight (GL_LIGHT1)
		GLfloat sunPosition[] = { 0.0f, 1.0f, 0.0f, 1.0f };
		GLfloat sunDiffuse[] = { 1.0f, 0.8f, 0.5f, 1.0f };
		GLfloat sunSpecular[] = { 1.0f, 0.8f, 0.5f, 1.0f };

		glPushMatrix();
		glRotatef(rotationAngle, 1, 0, 1);
		glLightfv(GL_LIGHT1, GL_POSITION, sunPosition);
		glLightfv(GL_LIGHT1, GL_DIFFUSE, sunDiffuse);
		glLightfv(GL_LIGHT1, GL_SPECULAR, sunSpecular);
		glPopMatrix();

		// Configure moonlight (GL_LIGHT2)
		GLfloat moonPosition[] = { 0.0f, -1.0f, 0.0f, 1.0f };
		GLfloat moonDiffuse[] = { 0.3f, 0.5f, 1.0f, 1.0f };
		GLfloat moonSpecular[] = { 0.3f, 0.5f, 1.0f, 1.0f };

		glPushMatrix();
		glRotatef(rotationAngle, 1, 0, 1);
		glLightfv(GL_LIGHT2, GL_POSITION, moonPosition);
		glLightfv(GL_LIGHT2, GL_DIFFUSE, moonDiffuse);
		glLightfv(GL_LIGHT2, GL_SPECULAR, moonSpecular);
		glPopMatrix();


		// Draw Map Model
		renderMap2();

		//sky box
		glPushMatrix();

		glScalef(150, 150, 150);

		glRotatef(rotationAngle, 1, 0, 1);
		model_skybox.Draw();

		glPopMatrix();

		DrawCrosshair(WIDTH, HEIGHT);

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
	else if (button == 'e') {
		if (currentMode == FIRST_PERSON) {
			currentMode = THIRD_PERSON;
		}
		else {
			currentMode = FIRST_PERSON;
		}
	}
	else if (button == 'm') {
		mouseEnabled = false;
		glutSetCursor(GLUT_CURSOR_LEFT_ARROW);
		if (currentDisplayMode != MAIN_MENU) {
			cleanupPhysicsWorld();
		}
		currentDisplayMode = MAIN_MENU;
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
	if (keyState[' ']) {
		if (abs(playerRigidBody->getLinearVelocity().y()) < 0.1f) {
			playerRigidBody->applyCentralImpulse(btVector3(0, 1000, 0)); // Jump impulse
		}
	}

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

void updateThirdPersonCamera() {
	// Define the offset from the player's position
	float distance = 5.0f;    // Distance behind the player
	float height = 0.0f;      // Base height above the player

	if (pitch > 89)
		pitch = 89;
	else if (pitch < -89)
		pitch = -89;

	// Calculate the spherical offset based on yaw and pitch
	Vector offset;
	offset.x = -distance * cos(radians(yaw)) * cos(radians(pitch)); // Horizontal distance
	offset.z = -distance * sin(radians(yaw)) * cos(radians(pitch)); // Depth distance
	offset.y = height - distance * sin(radians(pitch));             // Vertical offset

	// Set the camera position (Eye) to the player's position + offset
	Vector cameraPosition = Eye + offset;

	// Target is the player's position
	Vector target = Eye;


	// Update the view
	glLoadIdentity();
	gluLookAt(
		cameraPosition.x, cameraPosition.y, cameraPosition.z, // Camera position
		target.x, target.y, target.z,                        // Look at the player
		Up.x, Up.y, Up.z                   // Up vector
	);
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


	// First-person camera
	if (currentMode == FIRST_PERSON)
	{
		if (pitch > 89.0f) pitch = 89.0f;
		if (pitch < -89.0f) pitch = -89.0f;

		At.x = Eye.x + cos(radians(yaw)) * cos(radians(pitch));
		At.y = Eye.y + sin(radians(pitch));
		At.z = Eye.z + sin(radians(yaw)) * cos(radians(pitch));

		glLoadIdentity();
		gluLookAt(Eye.x, Eye.y, Eye.z, At.x, At.y, At.z, Up.x, Up.y, Up.z);
	}
	else // Third-person camera
	{
		updateThirdPersonCamera();
	}

	glutPostRedisplay();
}

void myMouse(int button, int state, int x, int y) {
	// Just set a flag or queue an event instead of doing heavy processing
	if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
		mouseState[GLUT_LEFT_BUTTON] = true;
		mouseEventX = x;
		mouseEventY = y;
	}
	else if (button == GLUT_LEFT_BUTTON && state == GLUT_UP) {
		mouseState[GLUT_LEFT_BUTTON] = false;
	}
	else if (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN) {
		mouseState[GLUT_RIGHT_BUTTON] = true;
	}
	else if (button == GLUT_RIGHT_BUTTON && state == GLUT_UP) {
		mouseState[GLUT_RIGHT_BUTTON] = false;
	}
}

btVector3 calculateRayTo(int x, int y, int WIDTH, int HEIGHT, const Vector& Eye, const Vector& forward, const Vector& right, const Vector& up)
{
	float fov = 60.0f; // Field of view
	float aspectRatio = static_cast<float>(WIDTH) / static_cast<float>(HEIGHT);

	// Normalize screen coordinates
	float nx = (2.0f * x) / WIDTH - 1.0f;
	float ny = 1.0f - (2.0f * y) / HEIGHT;

	// Calculate ray direction in camera space
	float tanFov = tan(3.141592653 * fov / 360.0f); // Convert FOV to radians
	Vector rayDir = (forward + right * (nx * tanFov * aspectRatio) + up * (ny * tanFov)).normalize();

	// Return the ray's destination in world space
	return Eye.toBtVector3() + rayDir.toBtVector3() * 1000.0f; // Scale ray length
}

void processMouseEvents() {
	if (mouseState[GLUT_LEFT_BUTTON] == true)
	{
		// Actual menu selection logic here
		float glX = (mouseEventX - WIDTH / 2.0f) * (WIDTH / (float)WIDTH);
		float glY = (HEIGHT / 2.0f - mouseEventY) * (HEIGHT / (float)HEIGHT);

		if (glX >= -250 && glX <= -50 && glY >= -75 && glY <= -25) {
			currentDisplayMode = MAP_1;
			initPhysicsWorld(1);
			mouseEnabled = true;
			glutSetCursor(GLUT_CURSOR_NONE);
			score = 0;
		}
		if (glX >= 50 && glX <= 250 && glY >= -75 && glY <= -25) {
			currentDisplayMode = MAP_2;
			initPhysicsWorld(2);
			mouseEnabled = true;
			glutSetCursor(GLUT_CURSOR_NONE);
			score = 0;
		}

		if (currentDisplayMode != MAIN_MENU) {
			// Calculate camera orientation
			Vector forward = (At - Eye).normalize();
			Vector right = forward.cross(Up).normalize();
			Vector up = right.cross(forward);

			// Compute ray
			btVector3 rayFrom = Eye.toBtVector3();
			btVector3 rayTo = calculateRayTo(mouseEventX, mouseEventY, WIDTH, HEIGHT, Eye, forward, right, up);

			// Perform raycast
			btCollisionWorld::ClosestRayResultCallback rayCallback(rayFrom, rayTo);
			dynamicsWorld->rayTest(rayFrom, rayTo, rayCallback);

			if (rayCallback.hasHit())
			{
				const btRigidBody* hitObject = btRigidBody::upcast(rayCallback.m_collisionObject);
				btVector3 hitPoint = rayCallback.m_hitPointWorld;

				const std::string* objectName = static_cast<const std::string*>(hitObject->getUserPointer());

				if (objectName)
				{
					printf("Hit object: %s at (%f, %f, %f)\n",
						objectName->c_str(),
						hitPoint.x(), hitPoint.y(), hitPoint.z());
				}
				else
				{
					printf("Hit object at: (%f, %f, %f)\n",
						hitPoint.x(), hitPoint.y(), hitPoint.z());
				}
			}
		}
	}
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
	// Request a redraw
	glutPostRedisplay();

	processMouseEvents();

    // Update physics simulation
    updatePhysics(1.0f / 144.0f); // Assuming 144 FPS

	//example collision check for testing
	//checkBulletCollision(playerRigidBody, mapRigidBody);

    updateMovement();

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