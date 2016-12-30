#pragma once

#include "vec3.h"
#include "matrix.h"

#include <vector>

using namespace std;

class Simulation;

struct Face
{
	Face(int a, int b, int c) : vertices{ a, b, c } {}
	Face(vector<int> vertices) : vertices(vertices) {}

	vector<int> vertices;

	vec3 normal;
	vec3 worldNormal;

};

struct Vertex
{
	Vertex(float x, float y, float z) : pos(x, y, z) {}
	Vertex(vec3 pos) : pos(pos) {}

	vec3 pos;
	vec3 worldPos;
};

struct Mesh {
	Mesh(Simulation* sim);
	Mesh(vector<Face>& faces, vector<Vertex>& vertices, Simulation* sim);

	void findNormals();
	void transformNormalsToWorld();

	void transformVerticesToWorld();
	vec3 transformToWorld(vec3& pos);

	void calculateInverseI();

	vec3 getX();
	matrix getR();
	vec3 getP();
	vec3 getL();

	vec3 getV();

	void setX(vec3 value);
	void setR(matrix value);
	void setP(vec3 value);
	void setL(vec3 value);

	matrix I0;
	matrix inverseI0;

	matrix inverseI;

	int stateIndex;

	vector<Face> faces;
	vector<Vertex> vertices;
	Simulation* sim;

	vec3 doGetState(int specific);
	void doSetState(vec3 value, int specific);

	bool resting = false;

	float mass;
};