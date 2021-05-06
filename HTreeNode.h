#pragma once
#include "vec.h"
#include <list>
typedef void (*DrawFuncT)();

class HTreeNode
{
public:
	const char* name;
	DrawFuncT drawFunc;
	std::list<HTreeNode*> children;
	Vec3d S;
	Vec3d T;
	Vec3d R;
	double angle;

	HTreeNode(const char* name, DrawFuncT drawFunc);
	void addChild(HTreeNode* n, Vec3d S, Vec3d T, Vec3d R, double angle);
	void draw(int depth);
};

