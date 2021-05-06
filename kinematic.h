#pragma once
#include "HTreeNode.h"
#include "vec.h"
#include "mat.h"

#include <vector>


#define IK_DELTA 0.0001
#define IK_EPSILON 0.000001
#define IK_MAX_IT 1000
#define IK_LR 1.0f

double distance(const Vec3d& a, const Vec3d& b);

Vec3d kForward(std::vector<HTreeNode*>& nodes);

class IKinematic
{
	HTreeNode* root;
	HTreeNode* end;
	std::vector<HTreeNode*> nodes;
	std::vector<double> constrainsLow;
	std::vector<double> constrainsHigh;
public:
	IKinematic();
	IKinematic(std::vector<HTreeNode*>& nodes);
	void solve(Vec3d target);
	void setConstrain(const char* name, double low, double high);
};

