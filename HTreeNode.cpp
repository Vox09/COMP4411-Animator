#include "modelerview.h"
#include "modelerapp.h"
#include "modelerdraw.h"
#include <FL/gl.h>

#include "modelerglobals.h"
#ifndef SAMPLE
#include "HTreeNode.h"

HTreeNode::HTreeNode(
		const char* n,
		DrawFuncT dF) :
	name(n), drawFunc(dF), S({ 1,1,1 }), T({ 0,0,0 }), R({ 0,0,1 }), angle(0) {};

void HTreeNode::addChild(HTreeNode* n,
		Vec3d S,
		Vec3d T,
		Vec3d R,
		double angle)
{
	children.push_back(n);
	children.back()->S = S;
	children.back()->T = T;
	children.back()->R = R;
	children.back()->angle = angle;
}

void HTreeNode::draw(int depth)
{
	if (depth >= VAL(LVL))
		return;
	Vec3d tmppS = S;
	Vec3d tmppT = T;
	Vec3d tmppR = R;
	
	glScaled(tmppS[0], tmppS[1], tmppS[2]);
	glTranslated(tmppT[0], tmppT[1], tmppT[2]);
	glRotated(angle, tmppR[0], tmppR[1], tmppR[2]);
	glPushMatrix();
	drawFunc();
	glPopMatrix();
	for (auto& child : children)
	{
		glPushMatrix();
		child->draw(depth + 1);
		glPopMatrix();
	}
}
#endif
