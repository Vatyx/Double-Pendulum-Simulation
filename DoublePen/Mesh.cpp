#include "Mesh.h"

#include "Simulation.h"

Mesh::Mesh(Simulation* sim) : sim(sim) {
}

Mesh::Mesh(vector<Face>& faces, vector<Vertex>& vertices, Simulation* sim) : faces(faces), vertices(vertices), sim(sim) {
}

void Mesh::findNormals()
{
	for (Face& face : faces)
	{
		auto a = vertices[face.vertices[0]].pos;
		auto b = vertices[face.vertices[1]].pos;
		auto c = vertices[face.vertices[2]].pos;

		auto normal = cross((b - a), (c - a));

		if (dot(normal, a) < 0)
			normal = -normal;

		normal.make_unit_vector();
		face.normal = normal;
	}
}

void Mesh::transformNormalsToWorld()
{
	for (Face& face : faces)
	{
		face.worldNormal = getR() * face.normal;
	}
}

void Mesh::transformVerticesToWorld()
{
	for (Vertex& vertex : vertices)
	{
		vertex.worldPos = transformToWorld(vertex.pos);
	}
}

vec3 Mesh::transformToWorld(vec3& v)
{
	return getX() + getR() * v;
}

void Mesh::calculateInverseI()
{
	auto& R = getR();
	auto transpose = R.transpose();
	
	auto one = inverseI0 * transpose;
	auto two = R * one;
	inverseI = R * (inverseI0 * transpose);
}

vec3 Mesh::doGetState(int specific)
{
	return sim->state[stateIndex + specific * sim->numMeshes];
}

void Mesh::doSetState(vec3 value, int specific)
{
	sim->state[stateIndex + specific * sim->numMeshes] = value;
}

vec3 Mesh::getX() {
	return doGetState(POSITION);
}

matrix Mesh::getR() {
	return sim->rotState[stateIndex];
}

vec3 Mesh::getP() {
	return doGetState(MOMENTUM);
}

vec3 Mesh::getL() {
	return doGetState(ANGULARMOM);
}

vec3 Mesh::getV() {
	return (1.0 / mass) * doGetState(MOMENTUM);
}

void Mesh::setP(vec3 value) {
	doSetState(value, MOMENTUM);
}

void Mesh::setL(vec3 value) {
	doSetState(value, ANGULARMOM);
}