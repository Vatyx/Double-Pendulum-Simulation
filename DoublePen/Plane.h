#pragma once

#include "vec3.h"

#include <vector>

using namespace std;

inline int largestComp(vec3 test)
{
	if (abs(test.x()) > abs(test.y()))
	{
		if (abs(test.x()) > abs(test.z()))
		{
			return 0; //x is biggest
		}
		else
		{
			return 2; //z is biggest;
		}
	}
	else
	{
		if (abs(test.y()) > abs(test.z()))
		{
			return 1; //y is biggest
		}
		else
		{
			return 2; //z is biggest
		}
	}
}

struct Plane {
	Plane(vec3 a, vec3 b, vec3 c)
	{
		point = a;
		normal = cross((b - a), (c - a));
		normal.make_unit_vector();

		largest = largestComp(normal);

		vertices.push_back(a);
		vertices.push_back(b);
		vertices.push_back(c);

		edgeVectors.push_back(b - a);
		edgeVectors.push_back(c - b);
		edgeVectors.push_back(a - c);
	}

	Plane(vec3 a, vec3 b, vec3 c, vec3 d)
	{
		point = a;
		normal = cross((b - a), (c - a));
		normal.make_unit_vector();

		largest = largestComp(normal);

		vertices.push_back(a);
		vertices.push_back(b);
		vertices.push_back(c);
		vertices.push_back(d);

		edgeVectors.push_back(b - a);
		edgeVectors.push_back(c - b);
		edgeVectors.push_back(d - c);
		edgeVectors.push_back(a - d);
	}

	vec3 point;
	vec3 normal;

	float restitution = .7;

	vector<vec3> vertices;
	vector<vec3> edgeVectors;

	int largest;
};