#include "CatmullromCurveEvaluator.h"
#include <assert.h>
#include "vec.h"
#include "mat.h"

void CatmullromCurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts,
	std::vector<Point>& ptvEvaluatedCurvePts,
	const float& fAniLength,
	const bool& bWrap) const 
{
	int iCtrlPtCount = ptvCtrlPts.size();
	ptvEvaluatedCurvePts.clear();

	float interval = 0.001;
	Mat4<float> M(-1, 3, -3, 1, 3, -6, 3, 0, -3, 3, 0, 0, 1, 0, 0, 0);
	std::vector<Point> ControlPoints;
	if (bWrap) {
		Point p1(ptvCtrlPts[iCtrlPtCount - 2].x - fAniLength, ptvCtrlPts[iCtrlPtCount - 2].y);
		Point p2(ptvCtrlPts[iCtrlPtCount - 1].x - fAniLength, ptvCtrlPts[iCtrlPtCount - 1].y);
		Point p3(ptvCtrlPts[0].x + fAniLength, ptvCtrlPts[0].y);
		Point p4(ptvCtrlPts[1].x + fAniLength, ptvCtrlPts[1].y);
		ControlPoints.push_back(p1);
		ControlPoints.push_back(p2);
		ControlPoints.insert(ControlPoints.end(), ptvCtrlPts.begin(), ptvCtrlPts.end());
		ControlPoints.push_back(p3);
		ControlPoints.push_back(p4);
	}
	else {
		Point p1(0, ptvCtrlPts[0].y);
		Point p2(fAniLength, ptvCtrlPts[iCtrlPtCount - 1].y);
		if (ptvCtrlPts.size() > 2) {
			ControlPoints.push_back(p1);
			ControlPoints.insert(ControlPoints.end(), ptvCtrlPts.begin(), ptvCtrlPts.end());
			ControlPoints.push_back(p2);
		}
	}
	for (vector<Point>::const_iterator i = ControlPoints.begin(); i != ControlPoints.end(); i++) {
		if ((i + 3) < ControlPoints.end()) {
			float Rx1 = (1.0 / 6) * ((double)(i + 2)->x - (double)i->x);
			float Rx2 = (1.0 / 6) * ((double)(i + 3)->x - (double)(i + 1)->x);
			float Ry1 = (1.0 / 6) * ((double)(i + 2)->y - (double)i->y);
			float Ry2 = (1.0 / 6) * ((double)(i + 3)->y - (double)(i + 1)->y);
			Vec4<float> Vx((i + 1)->x, (i + 1)->x + Rx1, (i + 2)->x - Rx2, (i + 2)->x);
			Vec4<float> Vy((i + 1)->y, (i + 1)->y + Ry1, (i + 2)->y - Ry2, (i + 2)->y);
			for (float t = 0; t <= 1; t += interval) {
				Vec4<float> T(t * t * t, t * t, t, 1);
				float Qx = T * (M * Vx);
				float Qy = T * (M * Vy);
				if ((ptvEvaluatedCurvePts.empty()) || (Qx > ptvEvaluatedCurvePts.back().x)) {
					ptvEvaluatedCurvePts.push_back(Point(Qx, Qy));
				}
			}
		}
	}
	if (!bWrap) {
		float x1 = 0.0;
		float y1 = ptvCtrlPts[0].y;
		ptvEvaluatedCurvePts.push_back(Point(x1, y1));

		float x2 = fAniLength;
		float y2 = ptvCtrlPts[iCtrlPtCount - 1].y;
		ptvEvaluatedCurvePts.push_back(Point(x2, y2));
	}
}