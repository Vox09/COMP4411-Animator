#pragma warning(disable : 4786)

#include "particleSystem.h"
#include "kinematic.h"


#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include <limits.h>


/***************
 * Constructors
 ***************/

ParticleSystem::ParticleSystem() 
{
	// TODO
	particles.push_back(new Particle());

}





/*************
 * Destructor
 *************/

ParticleSystem::~ParticleSystem() 
{
	// TODO
	for (auto& ptl : particles) {
		delete ptl;
	}
}


/******************
 * Simulation fxns
 ******************/

/** Start the simulation */
void ParticleSystem::startSimulation(float t)
{
	// TODO
	if (bake_end_time > 0)
		resetSimulation(t);
	std::cout << " start simulation at " << t << std::endl;
	bake_start_time = t;
	for (auto& ptl : particles) {
		ptl->start(t);
	}

	// These values are used by the UI ...
	// -ve bake_end_time indicates that simulation
	// is still progressing, and allows the
	// indicator window above the time slider
	// to correctly show the "baked" region
	// in grey.
	bake_end_time = -1;
	simulate = true;
	dirty = true;

}

/** Stop the simulation */
void ParticleSystem::stopSimulation(float t)
{
	std::cout << " stop simulation at " << t << std::endl;
	// TODO
	bake_end_time = t;
	for (auto& ptl : particles) {
		ptl->stop(t);
	}

	// These values are used by the UI
	simulate = false;
	dirty = true;

}

/** Reset the simulation */
void ParticleSystem::resetSimulation(float t)
{
	// TODO
	std::cout << "reset" << std::endl;
	for (auto& ptl : particles) {
		ptl->reset(t);
	}
	clearBaked();

	// These values are used by the UI
	simulate = false;
	dirty = true;
}

/** Compute forces and update particles **/
void ParticleSystem::computeForcesAndUpdateParticles(float t)
{
	// TODO
	if (simulate)
	{
		for (auto& ptl : particles)
		{
			ptl->update(t);
		}
	}
}


/** Render particles */
void ParticleSystem::drawParticles(float t)
{
	// TODO
	if (simulate)
	{
		for (auto& ptl : particles)
		{
			ptl->drawNew(t);
		}
		bakeParticles(t);
	}
	else
	{
		for (auto& ptl : particles)
		{
			ptl->drawBaked(t);
		}
	}
}





/** Adds the current configuration of particles to
  * your data structure for storing baked particles **/
void ParticleSystem::bakeParticles(float t) 
{
	// TODO
	for (auto& ptl : particles) {
		ptl->bake(t);
	}
}

/** Clears out your data structure of baked particles */
void ParticleSystem::clearBaked()
{
	// TODO
	for (auto& ptl : particles) {
		ptl->clear();
	}
}


void ParticleSystem::setNodes(int index, const vector<HTreeNode*>* nodes)
{
	if (index >= particles.size())
		return;
	particles[index]->setNodes(nodes);
}


