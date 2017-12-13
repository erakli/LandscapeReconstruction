#pragma once

#include "SimpleMappingCore.h"

namespace SimpleMapping 
{

	struct SceneParams
	{
		struct FeatureParams {
			int MAX_CORNERS = 1000;
			double qualityLevel = 0.3;
			double minDistance = 7.0;
			int blockSize = 7;
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
			int radius = 15;
			Scalar color = Scalar(0);
			int thickness = -1;
		} maskParams;
	};

}