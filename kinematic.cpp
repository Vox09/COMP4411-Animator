#include "kinematic.h"
#include "modelerglobals.h"

#include <cmath>
using std::vector;
using std::pair;

double distance(const Vec3d& a, const Vec3d& b)
{
	Vec3d tmp = a;
	tmp[0] -= b[0];
	tmp[1] -= b[1];
	tmp[2] -= b[2];
	return tmp.length2();
}

Mat3d kRotate(const double angle, const Vec3d& R_)
{
	Vec3d R = R_;
	R.normalize();
	double c = cos(angle * M_PI / 180.0f);
	double s = sin(angle * M_PI / 180.0f);
	return Mat3d(
		{
		c + (1 - c) * R[0] * R[0], (1 - c) * R[0] * R[1] - s * R[2], (1 - c) * R[0] * R[2] + s * R[1],
		(1 - c) * R[1] * R[0] + s * R[2], c + (1 - c) * R[1] * R[1], (1 - c) * R[1] * R[2] - s * R[0],
		(1 - c) * R[2] * R[0] - s * R[1], (1 - c) * R[2] * R[1] + s * R[0], c + (1 - c) * R[2] * R[2]
		}
	);
}

Vec3d kForward(std::vector<HTreeNode*>& nodes) {
	Vec3d rst = nodes.back()->T;
	for (int i = nodes.size()-1 ; i >=0; --i)
	{
		Mat3d rot = kRotate(nodes[i]->angle, nodes[i]->R);
		rst = rot * rst;
		rst = rst + nodes[i]->T;
	}
	return Vec3d(rst[0], rst[1], rst[2]);
}

IKinematic::IKinematic() {}

IKinematic::IKinematic(vector<HTreeNode*>& nodes_):
	nodes(nodes_)
{
	for (int i = 0; i < nodes.size(); ++i)
	{
		constrainsLow.push_back(0.0);
		constrainsHigh.push_back(0.0);
	}
}

void IKinematic::solve(Vec3d target)
{
	//kForward(nodes);
	double dist_before = distance(target, kForward(nodes));
	for (int itr = 0; itr < IK_MAX_IT; ++itr)
	{
		int n = nodes.size();
		// compute grads
		vector<double> grads(n);
		for (int i = 1; i < n; ++i)
		{
			nodes[i]->angle += IK_DELTA;
			double upper = distance(target, kForward(nodes));
			nodes[i]->angle -= IK_DELTA * 2;
			double lower = distance(target, kForward(nodes));
			nodes[i]->angle += IK_DELTA;
			grads[i] = 2 * (upper - lower) / IK_DELTA;
		}
		// update
		for (int i = 0; i < n; ++i)
		{
			double newAngle = nodes[i]->angle - grads[i] * IK_LR;
			double lowlimit = constrainsLow[i];
			double uplimit = constrainsHigh[i];
			if (uplimit > lowlimit)
			{
				newAngle = newAngle > uplimit ? uplimit : newAngle;
				newAngle = newAngle < lowlimit ? lowlimit : newAngle;
			}
			nodes[i]->angle = newAngle;
		}

		Vec3d endPoint = kForward(nodes);
		double dist_now = distance(target, endPoint);
		if (dist_now < IK_EPSILON || 
			(dist_now - dist_before < IK_EPSILON && dist_now - dist_before > -IK_EPSILON))
		{
			break;
		}
		dist_before = dist_now;
	}
}

void IKinematic::setConstrain(const char* name, double low, double high)
{
	for (int i=0 ; i < nodes.size(); ++i)
	{
		if (strcmp(nodes[i]->name, name) == 0)
		{
			constrainsLow[i] = low;
			constrainsHigh[i] = high;
		}
	}
}

