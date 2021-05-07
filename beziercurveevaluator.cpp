#include "BezierCurveEvaluator.h"
#include <assert.h>
#include "vec.h"
#include "mat.h"

void BezierCurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts,
	std::vector<Point>& ptvEvaluatedCurvePts,
	const float& fAniLength,
	const bool& bWrap) const {

	int iCtrlPtCount = ptvCtrlPts.size();
	ptvEvaluatedCurvePts.clear();

	float interval = 0.001;
	Mat4<float> M(-1, 3, -3, 1, 3, -6, 3, 0, -3, 3, 0, 0, 1, 0, 0, 0);
	int flag = 0;
	for (vector<Point>::const_iterator i = ptvCtrlPts.begin(); i != ptvCtrlPts.end(); ) {
		if (i < (ptvCtrlPts.end() - 3)) {
			Vec4<float> px((*i).x, (*(i + 1)).x, (*(i + 2)).x, (*(i + 3)).x);
			Vec4<float> py((*i).y, (*(i + 1)).y, (*(i + 2)).y, (*(i + 3)).y);
			for (float t = 0; t <= 1; t += interval) {
				Vec4<float> T(t * t * t, t * t, t, 1);
				float Qx = T * (M * px);
				float Qy = T * (M * py);
				ptvEvaluatedCurvePts.push_back(Point(Qx, Qy));
			}
			i += 3;
		}
		else if ((bWrap) && (i == (ptvCtrlPts.end() - 3))) {
			Vec4<float> px((*i).x, (*(i + 1)).x, (*(i + 2)).x, ptvCtrlPts[0].x + fAniLength);
			Vec4<float> py((*i).y, (*(i + 1)).y, (*(i + 2)).y, ptvCtrlPts[0].y);
			for (float t = 0; t <= 1; t += interval) {
				Vec4<float> T(t * t * t, t * t, t, 1);
				float Qx = T * (M * px);
				float Qy = T * (M * py);
				if (Qx < fAniLength) {
					ptvEvaluatedCurvePts.push_back(Point(Qx, Qy));
				}
				else {
					ptvEvaluatedCurvePts.push_back(Point(Qx - fAniLength, Qy));
				}
			}
			i = ptvCtrlPts.end();
			flag = 1;
		}
		else {
			ptvEvaluatedCurvePts.push_back(*i);
			i++;
		}
	}

	if ((bWrap)&&(!flag)) {
		float y = ( ptvCtrlPts[0].y * (fAniLength - ptvCtrlPts[iCtrlPtCount - 1].x) +
					ptvCtrlPts[iCtrlPtCount - 1].y * ptvCtrlPts[0].x) /
				  ( ptvCtrlPts[0].x + fAniLength - ptvCtrlPts[iCtrlPtCount - 1].x);
		float x1 = 0.0;
		float x2 = fAniLength;
		ptvEvaluatedCurvePts.push_back(Point(x1, y));
		ptvEvaluatedCurvePts.push_back(Point(x2, y));
	}
	else if (!bWrap){
		float x1 = 0.0;
		float y1 = ptvCtrlPts[0].y;
		ptvEvaluatedCurvePts.push_back(Point(x1, y1));

		float x2 = fAniLength;
		float y2 = ptvCtrlPts[iCtrlPtCount - 1].y;
		ptvEvaluatedCurvePts.push_back(Point(x2, y2));
	}
}