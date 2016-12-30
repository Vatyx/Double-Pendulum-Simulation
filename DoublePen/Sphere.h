#pragma once

#include "Mesh.h"

struct Sphere : public Mesh
{
	Sphere(int radius, bool t, vec3 conn, Sphere* sphereConn, float m, Simulation* sim)
		: Mesh(sim)
	{
		mass = m;

		I0(0, 0) = 1;
		I0(1, 1) = 1;
		I0(2, 2) = 1;

		inverseI0 = I0.inverse();

		top = t;
		connected = conn;
		sphereConnected = sphereConn;
	}

	bool top;
	vec3 connected;
	Mesh* sphereConnected;
};