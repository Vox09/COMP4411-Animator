#include "particle.h"
#include <FL/gl.h>

Particle::Particle():
	bake_start_time(-1),
	bake_end_time(-1),
	now_time(0)
{}

Particle::~Particle()
{
	now_frame.clear();
	clear();
}


void Particle::start(float t)
{
	bake_start_time = t;
	now_time = t;
}

void Particle::stop(float t)
{
	bake_end_time = t;
}

void Particle::reset(float t)
{
	bake_end_time = -1;
	now_frame.clear();
}

void Particle::update(float t)
{
	// delete old points
	while (now_frame.size() > 0 && now_frame.back().spawn_time + DISAPPEAR_TIME < t)
	{
		now_frame.pop_back();
	}
	// update current
	while (now_time < t)
	{
		for (auto& p : now_frame)
		{
			Vec3f f = force(p);
			p.update(f, EULER_EPS);
		}
		now_time += EULER_EPS;
	}
	// create new particles
	Mat4d transform = kTransform(*nodes);
	Vec3d pos;
	Vec3d vel(rand()/double(RAND_MAX), rand()/double(RAND_MAX), rand()/double(RAND_MAX));
	pos = transform * pos;
	vel = transform * vel;
	now_frame.push_front(PointObj(t, 0.001, pos, vel));
}

void Particle::drawNew(float t)
{
	// std::cout << "draw new " << t << std::endl;
	drawFrame(now_frame);
}

void Particle::drawBaked(float t)
{
	const Frame* f = nullptr;
	for (const auto& tf : cache)
	{
		if (tf.first - t > 0)
		{
			f = &(tf.second);
			// std::cout << "draw baked " << t << std::endl;
			drawFrame(*f);
			return;
		}
	}
	// std::cout << "draw baked not found" << t << std::endl;
}

void Particle::bake(float t)
{
	// already baked this timestamp
	if (!cache.empty() && cache.back().first == t)
		return;
	TimeFrame tf(t, now_frame);
	cache.push_back(tf);
}

void Particle::clear()
{
	for (TimeFrame& tf : cache) 
	{
		Frame f = tf.second;
		f.clear();
	}
	cache.clear();
	std::cout << "clear" << std::endl;
}

void Particle::setNodes(const vector<HTreeNode*>* ns)
{
	nodes = ns;
}


void Particle::drawFrame(const Frame& f)
{
	Mat4d t = kTransform(*nodes);
	GLdouble tmp[16];
	t.getGLMatrix(tmp);
	glPushMatrix();
		glPointSize(2.0f);
		glColor3b(255, 255, 255);
		glBegin(GL_POINTS);
		for (const auto& pt : f)
		{
			glVertex3f(pt.x, pt.y, pt.z);
		}
		glEnd();
	glPopMatrix();
}

Vec3f Particle::force(const PointObj& p)
{
	return Vec3f(0, -9.8 * p.mass, 0);
}


