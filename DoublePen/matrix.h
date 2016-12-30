#pragma once
#include <vector>

#include "vec3.h"

class matrix
{
public:
	matrix();
	float& operator()(size_t i, size_t j);
	float operator()(size_t i, size_t j) const;

	matrix inverse() const;
	matrix transpose() const;

	static matrix rotation(vec3& v, float theta);
	void renormalize();

	static matrix star(vec3& v);

private:
	size_t mRows;
	size_t mCols;

	float e[9]{ 0, 0, 0, 0, 0, 0, 0, 0, 0 };
};

inline matrix::matrix() : mRows(3), mCols(3) {}

inline float& matrix::operator()(size_t i, size_t j)
{
	return e[i * mCols + j];
}

inline float matrix::operator()(size_t i, size_t j) const
{
	return e[i * mCols + j];
}

inline matrix operator+(const matrix& m1, const matrix& m2)
{
	matrix ret;

	ret(0, 0) = m1(0, 0) + m2(0, 0);
	ret(0, 1) = m1(0, 1) + m2(0, 1);
	ret(0, 2) = m1(0, 2) + m2(0, 2);

	ret(1, 0) = m1(1, 0) + m2(1, 0);
	ret(1, 1) = m1(1, 1) + m2(1, 1);
	ret(1, 2) = m1(1, 2) + m2(1, 2);

	ret(2, 0) = m1(2, 0) + m2(2, 0);
	ret(2, 1) = m1(2, 1) + m2(2, 1);
	ret(2, 2) = m1(2, 2) + m2(2, 2);

	return ret;
}

inline matrix operator*(const matrix& m1, const matrix& m2)
{
	matrix m;

	m(0, 0) = m1(0, 0) * m2(0, 0) + m1(0, 1) * m2(1, 0) + m1(0, 2) * m2(2, 0);
	m(0, 1) = m1(0, 0) * m2(0, 1) + m1(0, 1) * m2(1, 1) + m1(0, 2) * m2(2, 1);
	m(0, 2) = m1(0, 0) * m2(0, 2) + m1(0, 1) * m2(1, 2) + m1(0, 2) * m2(2, 2);

	m(1, 0) = m1(1, 0) * m2(0, 0) + m1(1, 1) * m2(1, 0) + m1(1, 2) * m2(2, 0);
	m(1, 1) = m1(1, 0) * m2(0, 1) + m1(1, 1) * m2(1, 1) + m1(1, 2) * m2(2, 1);
	m(1, 2) = m1(1, 0) * m2(0, 2) + m1(1, 1) * m2(1, 2) + m1(1, 2) * m2(2, 2);

	m(2, 0) = m1(2, 0) * m2(0, 0) + m1(2, 1) * m2(1, 0) + m1(2, 2) * m2(2, 0);
	m(2, 1) = m1(2, 0) * m2(0, 1) + m1(2, 1) * m2(1, 1) + m1(2, 2) * m2(2, 1);
	m(2, 2) = m1(2, 0) * m2(0, 2) + m1(2, 1) * m2(1, 2) + m1(2, 2) * m2(2, 2);

	return m;
}

inline vec3 operator*(const matrix& m, const vec3& v)
{
	vec3 ret(0, 0, 0);

	ret.setX(m(0, 0) * v.x() + m(0, 1) * v.y() + m(0, 2) * v.z());
	ret.setY(m(1, 0) * v.x() + m(1, 1) * v.y() + m(1, 2) * v.z());
	ret.setZ(m(2, 0) * v.x() + m(2, 1) * v.y() + m(2, 2) * v.z());

	return ret;
}

inline matrix operator*(const matrix& m, const float& c)
{
	matrix ret;

	ret(0, 0) = m(0, 0) * c;
	ret(0, 1) = m(0, 1) * c;
	ret(0, 2) = m(0, 2) * c;

	ret(1, 0) = m(1, 0) * c;
	ret(1, 1) = m(1, 1) * c;
	ret(1, 2) = m(1, 2) * c;

	ret(2, 0) = m(2, 0) * c;
	ret(2, 1) = m(2, 1) * c;
	ret(2, 2) = m(2, 2) * c;

	return ret;
}

inline matrix operator*(const float& c, const matrix& m)
{
	matrix ret;

	ret(0, 0) = m(0, 0) * c;
	ret(0, 1) = m(0, 1) * c;
	ret(0, 2) = m(0, 2) * c;

	ret(1, 0) = m(1, 0) * c;
	ret(1, 1) = m(1, 1) * c;
	ret(1, 2) = m(1, 2) * c;

	ret(2, 0) = m(2, 0) * c;
	ret(2, 1) = m(2, 1) * c;
	ret(2, 2) = m(2, 2) * c;

	return ret;
}

inline matrix matrix::inverse() const
{
	auto& m = (*this);

	float det = m(0, 0) * (m(1, 1) * m(2, 2) - m(2, 1) * m(1, 2)) -
		m(0, 1) * (m(1, 0) * m(2, 2) - m(1, 2) * m(2, 0)) +
		m(0, 2) * (m(1, 0) * m(2, 1) - m(1, 1) * m(2, 0));

	float invdet = 1 / det;

	matrix minv; // inverse of matrix m
	minv(0, 0) = (m(1, 1) * m(2, 2) - m(2, 1) * m(1, 2)) * invdet;
	minv(0, 1) = (m(0, 2) * m(2, 1) - m(0, 1) * m(2, 2)) * invdet;
	minv(0, 2) = (m(0, 1) * m(1, 2) - m(0, 2) * m(1, 1)) * invdet;
	minv(1, 0) = (m(1, 2) * m(2, 0) - m(1, 0) * m(2, 2)) * invdet;
	minv(1, 1) = (m(0, 0) * m(2, 2) - m(0, 2) * m(2, 0)) * invdet;
	minv(1, 2) = (m(1, 0) * m(0, 2) - m(0, 0) * m(1, 2)) * invdet;
	minv(2, 0) = (m(1, 0) * m(2, 1) - m(2, 0) * m(1, 1)) * invdet;
	minv(2, 1) = (m(2, 0) * m(0, 1) - m(0, 0) * m(2, 1)) * invdet;
	minv(2, 2) = (m(0, 0) * m(1, 1) - m(1, 0) * m(0, 1)) * invdet;

	return minv;
}

inline matrix matrix::transpose() const
{
	auto& m = (*this);

	matrix minv; // transpose of matrix m
	minv(0, 0) = m(0, 0);
	minv(0, 1) = m(1, 0);
	minv(0, 2) = m(2, 0);
	minv(1, 0) = m(0, 1);
	minv(1, 1) = m(1, 1);
	minv(1, 2) = m(2, 1);
	minv(2, 0) = m(0, 2);
	minv(2, 1) = m(1, 2);
	minv(2, 2) = m(2, 2);

	return minv;
}

inline matrix matrix::rotation(vec3& axis, float theta)
{
	matrix m;
	auto x = axis.x();
	auto y = axis.y();
	auto z = axis.z();

	auto c = cos(theta);
	auto s = sin(theta);

	m(0, 0) = c + x * x * (1 - c);
	m(0, 1) = x * y * (1 - c) - z * s;
	m(0, 2) = x * z * (1 - c) + y * s;

	m(1, 0) = y * x * (1 - c) + z * s;
	m(1, 1) = c + y * y * (1 - c);
	m(1, 2) = y * z * (1 - c) - x * s;

	m(2, 0) = z * x * (1 - c) - y * s;
	m(2, 1) = z * y * (1 - c) + x * s;
	m(2, 2) = c + z * z * (1 - c);

	return m;
}

inline void matrix::renormalize()
{
	auto& m = *this;

	vec3 x(m(0, 0), m(0, 1), m(0, 2));
	vec3 y(m(1, 0), m(1, 1), m(1, 2));
	vec3 z(m(2, 0), m(2, 1), m(2, 2));

	auto error = dot(x, y);

	auto x_ort = x - (error / 2.0)*y;
	auto y_ort = y - (error / 2.0)*x;
	auto z_ort = cross(x_ort, y_ort);

	x_ort.make_unit_vector();
	y_ort.make_unit_vector();
	z_ort.make_unit_vector();

	m(0, 0) = x_ort.x();
	m(0, 1) = x_ort.y();
	m(0, 2) = x_ort.z();

	m(1, 0) = y_ort.x();
	m(1, 1) = y_ort.y();
	m(1, 2) = y_ort.z();

	m(2, 0) = z_ort.x();
	m(2, 1) = z_ort.y();
	m(2, 2) = z_ort.z();
}

inline matrix matrix::star(vec3& v)
{
	matrix m;

	m(0, 1) = -v.z();
	m(0, 2) = v.y();
	m(1, 2) = -v.x();

	m(1, 0) = v.z();
	m(2, 0) = -v.y();
	m(2, 1) = v.x();

	return m;
}