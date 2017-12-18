#pragma once

#include "SimpleMappingCore.h"

namespace SimpleMapping 
{

	struct SceneParams
	{
		size_t detectInterval = 30;

		struct FeatureParams {
			int MAX_CORNERS = 1800;
			double qualityLevel = 0.015;
			double minDistance = 20.0;
			int blockSize = 3;
			bool useHarrisDetector = false;
		} featureParams;

		struct CornerParams {
			Size subPixWinSize = Size(10, 10);
			Size zeroZone = Size(-1, -1);
		} cornerParams;

		struct OpticalFlowParams {
			Size winSize = Size(15, 15);
			int maxLevel = 2;
			TermCriteria criteria =
				TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 10, 0.03);
			int flags = 0;
			double minEigThreshold = 1e-4;
		} opticalFlowParams;

		struct MaskParams {
			int radius = 10;
			Scalar color = Scalar(0);
			int thickness = -1;
		} maskParams;

		
		// максимальное допустимое отклонение от среднего вектора скорости
		double THETA_MAX = 10.0 * (CV_PI / 180.0); // рад

		// коэффициент от 0 до 1 - максимальное отклонение от среднего значения скорости
		double NORM_EPS_COEFF = 0.4;
	};

}