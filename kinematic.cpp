#include "kinematic.h"
#include "modelerglobals.h"

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

Mat4d kTransform(const double angle, const Vec3d& R, const Vec3d& T)
{
	Vec3d nR = R;
	nR.normalize();
	double c = cos(angle * M_PI / 180.0f);
	double s = sin(angle * M_PI / 180.0f);
	return Mat4d(
		{
		c + (1 - c) * nR[0] * nR[0], (1 - c) * nR[0] * nR[1] - s * nR[2], (1 - c) * nR[0] * nR[2] + s * nR[1], T[0],
		(1 - c) * nR[1] * nR[0] + s * nR[2], c + (1 - c) * nR[1] * nR[1], (1 - c) * nR[1] * nR[2] - s * nR[0], T[1],
		(1 - c) * nR[2] * nR[0] - s * nR[1], (1 - c) * nR[2] * nR[1] + s * nR[0], c + (1 - c) * nR[2] * nR[2], T[2],
		0, 0, 0, 1
		}
	);

}

Mat4d kTransform(const std::vector<HTreeNode*>& nodes)
{
	Mat4d rst; // entity
	for (int i = nodes.size() - 1; i >= 0; --i)
	{
		Mat4d t = kTransform(nodes[i]->angle, nodes[i]->R, nodes[i]->T);
		rst = t * rst;
	}
	return rst;
}

Vec3d kForward(const std::vector<HTreeNode*>& nodes) {
	Vec3d rst;
	rst = kTransform(nodes) * rst;
	return rst;
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

