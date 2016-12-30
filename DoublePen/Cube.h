#pragma once

#include "Mesh.h"

struct Cube : public Mesh
{
	Cube(int len, float m, Simulation* sim)
		: Mesh(sim)
	{
		vertices.push_back(Vertex(len, len, len));  //0
		vertices.push_back(Vertex(-len, len, len));  //0
		vertices.push_back(Vertex(len, -len, len));  //0
		vertices.push_back(Vertex(len, len, -len));  //0
		vertices.push_back(Vertex(len, -len, -len));  //0
		vertices.push_back(Vertex(-len, -len, len));  //0
		vertices.push_back(Vertex(-len, len, -len));  //0
		vertices.push_back(Vertex(-len, -len, -len));  //0

		faces.push_back(Face(0, 2, 4));
		faces.push_back(Face(0, 4, 3));

		faces.push_back(Face(0, 1, 3));
		faces.push_back(Face(3, 1, 6));

		faces.push_back(Face(0, 1, 5));
		faces.push_back(Face(0, 5, 2));

		faces.push_back(Face(2, 5, 7));
		faces.push_back(Face(2, 7, 4));

		faces.push_back(Face(4, 6, 7));
		faces.push_back(Face(4, 6, 3));

		faces.push_back(Face(6, 1, 5));
		faces.push_back(Face(6, 7, 5));

		mass = m;

		I0(0, 0) = (mass / 12) * (2 * len * len);
		I0(1, 1) = (mass / 12) * (2 * len * len);
		I0(2, 2) = (mass / 12) * (2 * len * len);

		inverseI0 = I0.inverse();

		findNormals();
	}
};