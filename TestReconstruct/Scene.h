#pragma once

#include <vector>

#include "SimpleMappingCore.h"
#include "FeaturePoint.h"
#include "SceneParams.h"

namespace SimpleMapping 
{

	class Scene 
	{
		typedef vector< FeaturePoint::Point_t >	FeaturesPositions;
	public:
		Scene(double frameRate);
		~Scene();

		void FindNewFeatures(InputArray frameGray);
		void TrackFeatures(InputArray prevGray, InputArray frameGray, InputOutputArray visualFrame);
		void ShowFeaturesInfo(InputOutputArray visualFrame) const;

		vector< FeaturePoint > featurePoints_;

	private:
		Mat MaskExistingFeatures(const Size &frameSize) const;
		FeaturesPositions GetLastFeaturesPositions() const;
		vector<bool> GoodTrackedPoints(const FeaturesPositions &original, const FeaturesPositions &reverse) const;
		size_t GetActiveFecturePoints() const;

		enum FeatureStates { FeatureState_Normal, FeatureState_New, FeatureState_Bad };

		void DrawFeature(InputOutputArray visualFrame, const FeaturePoint &featurePoint, FeatureStates featureState) const;
		void DrawPoint(InputOutputArray visualFrame, const FeaturePoint &featurePoint, const Scalar &color) const;
		void DrawTrack(InputOutputArray visualFrame, const FeaturePoint &featurePoint, const Scalar &color) const;
		void DrawVelocity(InputOutputArray visualFrame, const FeaturePoint &featurePoint, const Scalar &color) const;

		void DrawText(InputOutputArray visualFrame, const string &str, const Point &pos) const;

		size_t featuresAdded_;
		SceneParams params_;
		size_t trackLength_;
		double frameDeltaTime_;
	};

}

