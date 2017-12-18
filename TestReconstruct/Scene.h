#pragma once

#include <vector>

#include "SimpleMappingCore.h"
#include "FeaturePoint.h"
#include "SceneParams.h"
#include "Camera.h"

namespace SimpleMapping 
{

	class Scene 
	{
		typedef vector< FeaturePoint::Point_t >	FeaturesPositions;
		typedef reference_wrapper<FeaturePoint>	FeaturePointRef;
	public:
		Scene(const Camera &camera, double frameRate);
		~Scene();

		Mat ProcessNewFrame(const Mat &inputFrame);
		void SaveWorldPoints();

	private:
		bool NeedToFindNewFeatures() const;
		void SaveFrame();

		void SaveWorldPoints(const vector< Point3d > &worldPoints);
		vector< Point3d > FindWorldPoints();
		Point3d EvalWorldVec(const FeaturePoint &featurePoint) const;

		void FindNewFeatures();
		void TrackFeatures();

		Mat MaskExistingFeatures(const Size &frameSize) const;
		FeaturesPositions GetLastFeaturesPositions() const;
		vector<bool> GoodTrackedPoints(const FeaturesPositions &original, const FeaturesPositions &reverse) const;

		vector< FeaturePointRef > GetLastAddedFeaturePoints();
		vector< FeaturePointRef > GetLastGoodFeaturePoints();
		vector< FeaturePointRef > GetActiveFeaturePoints();
		size_t GetActiveFeaturePointsCount() const;
		
		bool MarkBadFeatures();
		bool CheckFeatureBad(const Veloc_t &featureVeloc) const;
		
		void EvalMeanVeloc(const vector< FeaturePointRef > &activeFeaturePoints);

		void ShowFeaturesInfo() const;

		enum FeatureStates { FeatureState_Normal, FeatureState_New, FeatureState_Bad, FeatureState_Inactive };

		void DrawActiveFeature(const FeaturePoint &featurePoint, bool isNew) const;
		void DrawFeature(const FeaturePoint &featurePoint, FeatureStates featureState) const;

		void DrawPoint(const FeaturePoint &featurePoint, const Scalar &color) const;
		void DrawTrack(const FeaturePoint &featurePoint, const Scalar &color) const;
		void DrawVelocity(const FeaturePoint &featurePoint, const Scalar &color) const;

		void DrawText(const string &str, const Point &pos) const;

		Mat frameGray_;
		Mat prevGray_;
		Mat visualFrame_;

		size_t frameIdx_;
		size_t lastKeyFrame_;

		vector< FeaturePoint > featurePoints_;
		size_t featuresAdded_;

		struct {
			Veloc_t vec;
			double norm;
			double maxEps;
		} meanVeloc_;

		Camera camera_;

		SceneParams params_;
		size_t trackLength_;
		double frameDeltaTime_;
	};

}

