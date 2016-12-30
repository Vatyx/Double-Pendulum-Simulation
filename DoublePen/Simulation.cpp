#include "Simulation.h"

#include "Mesh.h"
#include "Sphere.h"

#include <GL/glut.h>
#include <iostream>
#include <chrono>
#include <future>
#include <random>

#include <Eigen/Dense>

using Eigen::MatrixXf;

Simulation::Simulation(int ts) : totalSteps(ts), rng(time(0))
{
	srand(time(nullptr));

	step = 0.1;
	currentCons1 = vec3(0, 0, 0);
	currentCons2 = vec3(0, 0, 0);
	airResistanceOn = false;

	sphereInit();

	planes.push_back(Plane(vec3(100, -100, 100), vec3(100, -100, -100), vec3(-100, -100, -100), vec3(-100, -100, 100))); //negative y
	//planes.push_back(Plane(vec3(120, -120, 120), vec3(120, -120, -120), vec3(-120, -120, -120), vec3(-120, -120, 120))); //negative y

	state.push_back(vec3(0, 0, 0));
	state.push_back(vec3(0, 0, 0));

	gravity = vec3(0, -9.8, 0);

	getColor(points1_color);
	getColor(points2_color);

	firsttime = false;

	len1 = 60;
	len2 = 60;
}

void Simulation::reset()
{
	state.clear();
	planes.clear();
	rotState.clear();
	meshes.clear();
	cons1 = vec3(0, 0, 0);
	cons2 = vec3(0, 0, 0);
	numMeshes = 0;

	points1.clear();
	points2.clear();

	currentCons1 = vec3(0, 0, 0);
	currentCons2 = vec3(0, 0, 0);

	sphereInit();
	planes.push_back(Plane(vec3(100, -100, 100), vec3(100, -100, -100), vec3(-100, -100, -100), vec3(-100, -100, 100))); //negative y

	state.push_back(vec3(0, 0, 0));
	state.push_back(vec3(0, 0, 0));

	gravity = vec3(0, -9.8, 0);

	getColor(points1_color);
	getColor(points2_color);

	glutPostRedisplay();
}

void Simulation::simulate() {
	std::cout << "Simulating!" << std::endl;

	for (int t = 0; t < totalSteps; t += step) {
		if (shouldReset)
		{
			std::lock_guard<std::mutex> lock(lock);
			reset();
			shouldReset = false;
		}

		float remain = step;

		auto future = std::async([] { std::this_thread::sleep_for(std::chrono::milliseconds(16)); });

		if (!paused)
		{

			setupConstraintForce(nullptr, vec3(0, 0, 0));

			vector<vec3> newState = KN4NumInt(state);
			vector<matrix> newStateRot = KN4NumIntMatrix(rotState);

			state = newState;
			rotState = newStateRot;

			auto what = meshes[0]->getP().length();

			if (meshes[0]->getP().length() > 100)
			{
				auto newP = meshes[0]->getP();
				newP.make_unit_vector();
				newP *= 95;
				meshes[0]->setP(newP);

				if (!airResistanceOn)
				{
					//airResistanceOn = true;
				}
			}

			if (meshes[1]->getP().length() > 100)
			{
				auto newP = meshes[1]->getP();
				newP.make_unit_vector();
				newP *= 95;
				meshes[1]->setP(newP);

				/*if (!airResistanceOn)
				{
					airResistanceOn = true;
					std::async([&] {
						std::this_thread::sleep_for(std::chrono::milliseconds(160));
						airResistanceOn = false;
					});
				}*/
			}

			for (Mesh* mesh : meshes)
			{
				mesh->transformVerticesToWorld();
				mesh->transformNormalsToWorld();
				mesh->calculateInverseI();
			}

			points1.push_back(make_pair(meshes[0]->getX(), vec3(points1_color[0], points1_color[1], points1_color[2])));
			points2.push_back(make_pair(meshes[1]->getX(), vec3(points2_color[0], points2_color[1], points2_color[2])));

			points1_color[0] += points1_step[0];
			points1_color[1] += points1_step[1];
			points1_color[2] += points1_step[2];

			points2_color[0] += points2_step[0];
			points2_color[1] += points2_step[1];
			points2_color[2] += points2_step[2];

			if (colorsteps == 100)
			{
				colorsteps = 0;

				points1_prev[0] = points1_next[0];
				points1_prev[1] = points1_next[1];
				points1_prev[2] = points1_next[2];

				points2_prev[0] = points2_next[0];
				points2_prev[1] = points2_next[1];
				points2_prev[2] = points2_next[2];

				getColor(points1_next);
				getColor(points2_next);

				points1_step[0] = (points1_next[0] - points1_prev[0]) / 100.0;
				points1_step[1] = (points1_next[1] - points1_prev[1]) / 100.0;
				points1_step[2] = (points1_next[2] - points1_prev[2]) / 100.0;

				points2_step[0] = (points2_next[0] - points2_prev[0]) / 100.0;
				points2_step[1] = (points2_next[1] - points2_prev[1]) / 100.0;
				points2_step[2] = (points2_next[2] - points2_prev[2]) / 100.0;
			}


			auto kinetic = .5 * dot(meshes[0]->getV(), meshes[0]->getV()) + .5 * dot(meshes[1]->getV(), meshes[1]->getV());
			auto potential = 9.8 * (meshes[0]->getX().y() - (-50)) + 9.8 * (meshes[1]->getX().y() - (-50));

			cout << "Total Energy: " << (kinetic + potential) << endl;

			colorsteps++;
		}

		future.get();
		glutPostRedisplay();
	}
}

void Simulation::getColor(GLfloat* temp)
{
	temp[0] = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
	temp[1] = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
	temp[2] = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
}

void Simulation::sphereInit()
{
	matrix rot = matrix::rotation(vec3(1, 1, 1), 0);
	rot.renormalize();

	if (firsttime)
	{
		addSphere(20, true, vec3(0, 120, 0), nullptr, 1, vec3(0, 40, 0), rot, vec3(20, 0, 0), vec3(0, 0, 0));
		addSphere(20, false, vec3(0, 40, 0), meshes[0], 1, vec3(0, -40, 0), rot, vec3(-20, 0, 0), vec3(0, 0, 0));
	}
	else
	{
		auto a = static_cast <float> (rand()) / static_cast <float> (RAND_MAX) * 80 - 40;
		auto b = static_cast <float> (rand()) / static_cast <float> (RAND_MAX) * 80 - 40;
		auto c = static_cast <float> (rand()) / static_cast <float> (RAND_MAX) * 80 - 40;
		auto d = static_cast <float> (rand()) / static_cast <float> (RAND_MAX) * 80 - 40;
		addSphere(20, true, vec3(0, 120, 0), nullptr, 1, vec3(0, 40, 0), rot, vec3(a, 0, b), vec3(0, 0, 0));
		addSphere(20, false, vec3(0, 40, 0), meshes[0], 1, vec3(0, -40, 0), rot, vec3(c, 0, d), vec3(0, 0, 0));
	}
}


void Simulation::addSphere(float radius, bool t, vec3 conn, Mesh* sphereConn, float mass, vec3 x, matrix rot, vec3 v, vec3 o)
{
	Sphere* c = new Sphere(radius, t, conn, (Sphere*)sphereConn, 1, this);
	addMesh(c, x, rot, v, o);
}

void Simulation::addMesh(Mesh* mesh, vec3 x, matrix rot, vec3 v, vec3 o)
{
	mesh->stateIndex = nextMeshIndex();

	meshes.push_back(mesh);

	int index = mesh->stateIndex;

	insertState(POSITION, index, x);
	insertState(MOMENTUM, index, mesh->mass * v);
	insertState(ANGULARMOM, index, mesh->I0 * o);

	rotState.push_back(rot);

	mesh->transformVerticesToWorld();
	mesh->transformNormalsToWorld();

	mesh->calculateInverseI();
}

void Simulation::insertState(StateType type, int index, vec3 value)
{
	if (state.size() == 0)
		state.push_back(value);
	else
		state.insert(state.begin() + ((numMeshes * type) + index), value);
}

int Simulation::nextMeshIndex()
{
	return numMeshes++;
}

vector<vec3> Simulation::evaluation(vector<vec3> states, float step, vector<vec3> der)
{
	vector<vec3> evalState;
	for (int i = 0; i < states.size(); i++)
	{
		evalState.push_back(states[i] + der[i] * step);
	}

	vector<vec3> acc = acceleration(evalState, step);

	return acc;
}

vector<matrix> Simulation::matrixEvaluation(vector<matrix> states, float step, vector<matrix> der)
{
	vector<matrix> evalState;
	for (int i = 0; i < states.size(); i++)
	{
		evalState.push_back(states[i] + der[i] * step);
	}

	vector<matrix> acc = matrixAcceleration(evalState, step);

	return acc;
}

vector<vec3> Simulation::acceleration(vector<vec3> states, float step)
{
	vector<vec3> accelerations;

	//get change in position - velocity
	for (Mesh* mesh : meshes)
	{
		accelerations.push_back((1.0 / mesh->mass) * mesh->getP());
	}

	//get change in linear momentum - force
	for (Mesh* mesh : meshes)
	{
		accelerations.push_back(vec3(getExternalForces((Sphere*)mesh, mesh->getX())));
	}

	//get change in angular momentum - torque
	for (Mesh* mesh : meshes)
	{
		//accelerations.push_back(vec3(0, 0, 0)); //most of the time it is 0
		accelerations.push_back(getExternalTorque(mesh, mesh->getL()));
	}

	accelerations.push_back(currentCons1);
	accelerations.push_back(currentCons2);

	return accelerations;
}

vector<matrix> Simulation::matrixAcceleration(vector<matrix> states, float step)
{
	vector<matrix> accelerations;

	//get change in rotation
	for (Mesh* mesh : meshes)
	{
		auto R = mesh->getR();
		auto L = mesh->getL();
		auto star = matrix::star(mesh->inverseI * L);
		auto newR = star * R;

		accelerations.push_back(newR);
	}

	return accelerations;
}

vector<vec3> Simulation::KN4NumInt(vector<vec3>& states)
{
	vector<vec3> a, b, c, d;
	vector<vec3> newState;

	a = evaluation(states, 0, vector<vec3>(states.size()));
	b = evaluation(states, 0.5 * step, a);
	c = evaluation(states, 0.5 * step, b);
	d = evaluation(states, step, c);


	for (int i = 0; i < states.size(); i++)
	{
		newState.push_back(states[i] + step*(a[i] + 2 * b[i] + 2 * c[i] + d[i]) / 6.0);
	}

	return newState;
}

vector<matrix> Simulation::KN4NumIntMatrix(vector<matrix>& states)
{
	vector<matrix> a, b, c, d;
	vector<matrix> newState;

	a = matrixEvaluation(rotState, 0, vector<matrix>(rotState.size()));
	b = matrixEvaluation(states, 0.5 * step, a);
	c = matrixEvaluation(states, 0.5 * step, b);
	d = matrixEvaluation(states, step, c);

	for (int i = 0; i < rotState.size(); i++)
	{
		auto test = step*(a[i] + 2 * b[i] + 2 * c[i] + d[i]) * (1.0 / 6.0);
		auto ns = rotState[i] + test;
		ns.renormalize();
		newState.push_back(ns);
	}

	return newState;
}

vec3 Simulation::getExternalForces(Sphere* sphere, vec3& position)
{
	vec3 total(0, 0, 0);

	total += gravity;

	if(airResistanceOn)
		total += -sphere->getV() * 0.1;

	total += getConstraintForce(sphere, total);

	return total;
}

void Simulation::setupConstraintForce(Sphere* sphere, vec3& fa)
{
	/*MatrixXd m(9, 9);
	for (int i = 0; i < 9; i++)
	{
		for (int j = 0; j < 9; j++)
		{
			if (i == j)
			{
				m(i, j) = 1.0 / sphere->mass;
			}
			else
			{
				m(0, 0) = 0;
			}
		}
	}*/

	Sphere* top = (Sphere*)meshes[0];
	Sphere* bot = (Sphere*)meshes[1];

	MatrixXf J(2, 6);
	auto minusx1 = (top->getX() - vec3(0, 120, 0));
	auto x1 = (top->getX() - vec3(0, 120, 0));
	auto minusx2x1 = -(bot->getX() - top->getX());
	auto x2x1 = (bot->getX() - top->getX());

	J(0, 0) = minusx1.x();
	J(0, 1) = minusx1.y();
	J(0, 2) = minusx1.z();

	J(0, 3) = 0;
	J(0, 4) = 0;
	J(0, 5) = 0;

	J(1, 0) = minusx2x1.x();
	J(1, 1) = minusx2x1.y();
	J(1, 2) = minusx2x1.z();

	J(1, 3) = x2x1.x();
	J(1, 4) = x2x1.y();
	J(1, 5) = x2x1.z();

	MatrixXf Jdot(2, 6);
	auto minusv1 = (top->getV());
	auto minusv2v1 = -(bot->getV() - top->getV());
	auto v2v1 = (bot->getV() - top->getV());

	Jdot(0, 0) = minusv1.x();
	Jdot(0, 1) = minusv1.y();
	Jdot(0, 2) = minusv1.z();

	Jdot(0, 3) = 0;
	Jdot(0, 4) = 0;
	Jdot(0, 5) = 0;

	Jdot(1, 0) = minusv2v1.x();
	Jdot(1, 1) = minusv2v1.y();
	Jdot(1, 2) = minusv2v1.z();

	Jdot(1, 3) = v2v1.x();
	Jdot(1, 4) = v2v1.y();
	Jdot(1, 5) = v2v1.z();

	Eigen::VectorXf Xdot(6);

	Xdot(0) = top->getV().x();
	Xdot(1) = top->getV().y();
	Xdot(2) = top->getV().z();

	Xdot(3) = bot->getV().x();
	Xdot(4) = bot->getV().y();
	Xdot(5) = bot->getV().z();

	vec3 fa1 = gravity;// - 0.05 * top->getV();
	vec3 fa2 = gravity;// -0.05 * bot->getV();

	if (airResistanceOn)
	{
		fa1 += -0.05 * top->getV();
		fa2 += -0.05 * bot->getV();
	}

	Eigen::VectorXf Fa(6);

	Fa(0) = fa1.x();
	Fa(1) = fa1.y();
	Fa(2) = fa1.z();

	Fa(3) = fa2.x();
	Fa(4) = fa2.y();
	Fa(5) = fa2.z();

	Eigen::VectorXf C(2);
	C(0) = 1.0 / 2.0 * dot(top->getX() - vec3(0, 120, 0), top->getX() - vec3(0, 120, 0)) - (len1 * len1);
	C(1) = 1.0 / 2.0 * dot(bot->getX() - top->getX(), bot->getX() - top->getX()) - (len2 * len2);
	currentCons1 = vec3(C(0), 0, 0);
	currentCons2 = vec3(C(1), 0, 0);

	Eigen::VectorXf Cdot = J * Xdot;

	Eigen::VectorXf Cerr(2);
	Cerr(0) = state[state.size() - 2].x();
	Cerr(1) = state[state.size() - 1].x();

	MatrixXf leftside = J * J.transpose();
	auto lr = leftside.rows();
	auto lc = leftside.cols();

	Eigen::VectorXf rightside = -Jdot * Xdot - J * Fa - 50 * C - 20 * Cdot - 100 * Cerr;
	auto rr = rightside.rows();
	auto rc = rightside.cols();

	Eigen::VectorXf lambda = leftside.ldlt().solve(rightside);
	auto what = lambda.rows();
	auto who = lambda.cols();

	Eigen::VectorXf cons = J.transpose() * lambda;

	cons1 = vec3(cons(0), cons(1), cons(2));
	cons2 = vec3(cons(3), cons(4), cons(5));

	/*vec3 x0;
	if (sphere->top)
		x0 = sphere->connected;
	else
		x0 = sphere->sphereConnected->getX();

	vec3 x = sphere->getX();
	vec3 xdot = sphere->getV();

	auto lambda = -(sphere->mass * dot(xdot, xdot) + dot(x - x0, fa)) / dot(x - x0, x - x0);
	
	return lambda * (x - x0);*/
}

vec3 Simulation::getConstraintForce(Sphere* sphere, vec3 & position)
{
	if (sphere->top)
		return cons1;
	else
		return cons2;
}

vec3 Simulation::getExternalTorque(Mesh* mesh, vec3& angMom)
{
	vec3 total(0, 0, 0);

	return total;
}

int Simulation::detectDynamicCollision(vec3& position, vec3& velocity, Mesh* mesh)
{
	for (int i = 0; i < mesh->faces.size(); i++)
	{
		Plane plane(mesh->vertices[mesh->faces[i].vertices[0]].worldPos,
			mesh->vertices[mesh->faces[i].vertices[1]].worldPos,
			mesh->vertices[mesh->faces[i].vertices[2]].worldPos);
		plane.normal = mesh->faces[i].worldNormal;

		auto t = dot((plane.point - position), plane.normal) / dot(velocity, plane.normal);
		bool hit = true;
		if (0 < t && t < step)
		{
			auto x = position + t * velocity;
			bool first = true;
			bool sign = false;
			for (int j = 0; j < plane.vertices.size(); j++)
			{
				auto tempEdgeVec = x - plane.vertices[j];
				float det = 0;
				if (plane.largest == 0)
				{
					det = plane.edgeVectors[j].y() * tempEdgeVec.z() - plane.edgeVectors[j].z() * tempEdgeVec.y();
				}
				else if (plane.largest == 1)
				{
					det = (plane.edgeVectors[j].x() * tempEdgeVec.z()) - (plane.edgeVectors[j].z() * tempEdgeVec.x());
				}
				else if (plane.largest == 2)
				{
					det = plane.edgeVectors[j].x() * tempEdgeVec.y() - plane.edgeVectors[j].y() * tempEdgeVec.x();
				}

				if (first)
				{
					sign = det > 0;
					first = false;
				}
				else
				{
					if (sign != det > 0)
					{
						hit = false;
						break; //doesn't collide
					}
				}
			}

			if (hit)
			{
				return i; //it hits it so return the plane;
			}
		}
	}

	return -1;
}

int Simulation::detectStaticCollision(vec3& position, vec3& velocity)
{
	for (int i = 0; i < planes.size(); i++)
	{
		auto t = dot((planes[i].point - position), planes[i].normal) / dot(velocity, planes[i].normal);
		bool hit = true;
		if (0 < t && t < step)
		{
			auto x = position + t * velocity;
			bool first = true;
			bool sign = false;
			for (int j = 0; j < planes[i].vertices.size(); j++)
			{
				auto tempEdgeVec = x - planes[i].vertices[j];
				float det = 0;
				if (planes[i].largest == 0)
				{
					det = planes[i].edgeVectors[j].y() * tempEdgeVec.z() - planes[i].edgeVectors[j].z() * tempEdgeVec.y();
				}
				else if (planes[i].largest == 1)
				{
					det = (planes[i].edgeVectors[j].x() * tempEdgeVec.z()) - (planes[i].edgeVectors[j].z() * tempEdgeVec.x());
				}
				else if (planes[i].largest == 2)
				{
					det = planes[i].edgeVectors[j].x() * tempEdgeVec.y() - planes[i].edgeVectors[j].y() * tempEdgeVec.x();
				}

				if (first)
				{
					sign = det > 0;
					first = false;
				}
				else
				{
					if (sign != det > 0)
					{
						hit = false;
						break; //doesn't collide
					}
				}
			}

			if (hit)
			{
				return i; //it hits it so return the plane;
			}
		}
	}

	return -1;
}

void Simulation::handleStaticCollision(int collided, Mesh* mesh, vec3& r, vec3& velocity)
{
	auto collidedPlane = planes[collided];
	float res = collidedPlane.restitution;
	auto normal = collidedPlane.normal;

	auto velneg = dot(velocity, collidedPlane.normal);

	auto num = -(1 + res) * velneg;
	float invmass = 1.0 / mesh->mass;

	auto j = num / (invmass + dot(normal, cross(mesh->inverseI * cross(r, normal), r)));

	auto J = j * normal;

	mesh->setP(mesh->getP() + J);
	mesh->setL(mesh->getL() +  j * cross(r, normal) );
}

void Simulation::handleDynamicCollision(Mesh* a, Mesh* b, vec3& va, vec3& vb, vec3& ra, vec3& rb, vec3& normal)
{
	float res = 0.9;

	auto velrel = dot(normal, va - vb);

	auto num = -(1 + res) * velrel;
	float ima = 1.0 / a->mass;
	float imb = 1.0 / b->mass;

	auto j = num / (ima + imb + dot(normal, 
		cross(a->inverseI * cross(ra, normal), ra) + 
		cross(b->inverseI * cross(rb, normal), rb)));

	auto J = j * normal;

	a->setP(a->getP() + J);
	a->setL(a->getL() +  j * cross(ra, normal) );

	b->setP(b->getP() - J);
	b->setL(b->getL() -  j * cross(rb, normal) );
}