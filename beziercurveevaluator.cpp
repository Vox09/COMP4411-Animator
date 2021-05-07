#include "BezierCurveEvaluator.h"
#include <assert.h>
#include "vec.h"
#include "mat.h"

void BezierCurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts,
	std::vector<Point>& ptvEvaluatedCurvePts,
	const float& fAniLength,
	const bool& bWrap) const {
	float interval = 0.1;
	Mat4<float> M(-1, 3, -3, 1, 3, -6, 3, 0, -3, 3, 0, 0, 1, 0, 0, 0);
	for (vector<Point>::const_iterator i = ptvCtrlPts.begin(); i != ptvCtrlPts.end(); i++) {
		if ((i + 3) < ptvCtrlPts.end()) {
			Vec4<float> px((*i).x, (*(i + 1)).x, (*(i + 2)).x, (*(i + 3)).x);
			Vec4<float> py((*i).y, (*(i + 1)).y, (*(i + 2)).y, (*(i + 3)).y);
			for (float t = 0; t <= 1; t += interval) {
				Vec4<float> T(t * t * t, t * t, t, 1);
				float Qx = (T * M) * px;
				float Qy = (T * M) * py;
				ptvEvaluatedCurvePts.push_back(Point(Qx, Qy));
			}
		}
		else {
			ptvEvaluatedCurvePts.push_back(*i);
		}
	}
}