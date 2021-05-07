#include "pointObj.h"

void PointObj::updateA(Vec3f force)
{
	ax = force[0] / mass;
	ay = force[1] / mass;
	az = force[2] / mass;
}

void PointObj::updateV(double time_eps)
{
	vx += ax * time_eps;
	vy += ay * time_eps;
	vz += az * time_eps;
}

void PointObj::updatePos(double time_eps)
{
	x += vx * time_eps;
	y += vy * time_eps;
	z += vz * time_eps;
}

void PointObj::update(Vec3f force, double time_eps)
{
	updateA(force);
	updateV(time_eps);
	updatePos(time_eps);
}


