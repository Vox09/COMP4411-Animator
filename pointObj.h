#ifndef POINTOBJ_H
#define POINTOBJ_H

#include "point.h"
#include "vec.h"

class PointObj : private Point {

	float z;
	float spawn_time;
	float mass;

	float vx, vy, vz;
	float ax, ay, az;

	void updateA(Vec3f force);
	void updateV(double time_eps);
	void updatePos(double time_eps);
	
public:
	PointObj(float t, float mass, Vec3d pos, Vec3d vel) :
		spawn_time(t),
		mass(mass),
		z(pos[2]),
		vx(vel[0]),
		vy(vel[1]),
		vz(vel[2]),
		ax(0.0),
		ay(0.0),
		az(0.0)
	{
		// in Point class
		x = pos[0];
		y = pos[1];
	};

	void update(Vec3f force, double time_eps);

	friend class Particle;
};

#endif // !POINTOBJ_H

