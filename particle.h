#ifndef PARTICLE_H
#define PARTICLE_H

#include "pointObj.h"
#include <list>
#include <set>
#include "kinematic.h"

#define EULER_EPS 0.001

#define DISAPPEAR_TIME	3.0f
#define APPEAR_TIME		3.0f

using std::list;
using std::pair;

typedef list<PointObj> Frame;
typedef pair<float, Frame> TimeFrame;

class Particle
{
	float bake_start_time;
	float bake_end_time;
	list<TimeFrame> cache;
	Frame now_frame;
	float now_time;
	const vector<HTreeNode*>* nodes;
	 
	void drawFrame(const Frame& frame);
	Vec3f force(const PointObj& point);
public:
	Particle();
	~Particle();
	void start(float time);
	void stop(float time);
	void reset(float time);
	void update(float time);
	void drawNew(float time);
	void drawBaked(float time);
	void bake(float time);
	void clear();

	void setNodes(const vector<HTreeNode*>* nodes);
};

#endif // PARTICLE_H
