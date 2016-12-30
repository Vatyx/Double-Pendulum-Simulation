#pragma once

#include "Plane.h"

#include <GL/glut.h>
#include <vector>
#include <utility>
#include <mutex>
#include <chrono>
#include <random>

#include "matrix.h"
#include "vec3.h"

const auto pi = 3.14159265358979323846;

enum StateType {
	POSITION,
	MOMENTUM,
	ANGULARMOM,
};

class Mesh;
class Sphere;
class Vertex;
class Face;

class Simulation {
public:
	Simulation(int ts);

	void simulate();

	void reset();

	void getColor(GLfloat* temp);

	void sphereInit();

	void cubeSpawn();

	void spawnCubeCollision();

	void addSphere(float radius, bool t, vec3 conn, Mesh * sphereConn, float mass, vec3 x, matrix rot, vec3 v, vec3 o);
	void addMesh(Mesh* mesh, vec3 pos, matrix rot, vec3 vel, vec3 omg);

	void insertState(StateType type, int index, vec3 value);

	int nextMeshIndex();

	vec3 getExternalForces(vec3& position);
	vec3 getExternalTorque(Mesh * mesh, vec3 & angMom);
	vector<vec3> Simulation::computeAccelerations();
	vector<vec3> Simulation::NumInt(vector<vec3>& stateUpdates);

	vector<vec3> evaluation(vector<vec3> states, float step, vector<vec3> der);
	vector<vec3> acceleration(vector<vec3> states, float step);
	vector<matrix> matrixEvaluation(vector<matrix> states, float step, vector<matrix> der);
	vector<matrix> matrixAcceleration(vector<matrix> states, float step);

	vector<vec3> KN4NumInt(vector<vec3>& stateUpdates);
	vector<matrix> KN4NumIntMatrix(vector<matrix>& stateUpdates);

	vec3 getExternalForces(Sphere* mesh, vec3 & position);
	void setupConstraintForce(Sphere * sphere, vec3 & fa);
	vec3 getConstraintForce(Sphere* mesh, vec3 & position);

	int detectStaticCollision(vec3& position, vec3& velocity);
	int detectDynamicCollision(vec3& position, vec3& velocity, Mesh* mesh);
	void handleStaticCollision(int collided, Mesh* mesh, vec3& r, vec3& velocity);
	void handleDynamicCollision(Mesh* a, Mesh* b, vec3& va, vec3& vb, vec3& ra, vec3& rb, vec3& normal);

	vector<vec3> state;
	vector<matrix> rotState;
	int numMeshes = 0;

	vector<Mesh*> meshes;
	vector<Plane> planes;

	vec3 gravity;

	mutex stateMutex;
	std::mt19937 rng;

	float restitution;
	float friction;

	vec3 cons1;
	vec3 cons2;
	vec3 currentCons1;
	vec3 currentCons2;

	bool airResistanceOn;

	std::vector<pair<vec3, vec3>> points1;
	std::vector<pair<vec3, vec3>> points2;

	GLfloat points1_color[3] = { 0, 0, 0 };
	GLfloat points1_step[3] = { 0, 0, 0 };
	GLfloat points2_color[3] = { 0, 0, 0 };
	GLfloat points2_step[3] = { 0, 0, 0 };

	GLfloat points1_prev[3] = { 0, 0, 0 };
	GLfloat points2_prev[3] = { 0, 0, 0 };
	GLfloat points1_next[3] = { 0, 0, 0 };
	GLfloat points2_next[3] = { 0, 0, 0 };

	unsigned long colorsteps = 0;
	
	bool shouldReset = true;
	bool firsttime = true;

	std::mutex lock;

	float len1;
	float len2;

	bool draw1 = true;
	bool draw2 = true;

	bool paused = false;
	bool showballs = true;

private:
	float step;
	int totalSteps;
};