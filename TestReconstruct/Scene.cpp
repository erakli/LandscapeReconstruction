#include <opencv2/imgproc.hpp>

#include "Scene.h"
#include <opencv2/video/tracking.hpp>

using namespace SimpleMapping;


Scene::Scene(double frameRate)
	: featuresAdded_(0)
	, trackLength_(frameRate)
	, frameDeltaTime_(1.0 / frameRate)
{
}

Scene::~Scene()
{
}



void Scene::FindNewFeatures(InputArray frameGray) 
{
	Mat mask = MaskExistingFeatures(frameGray.size());

	auto &featureParams = params_.featureParams;

	FeaturesPositions newPoints;
	goodFeaturesToTrack(
		frameGray, newPoints,
		featureParams.MAX_CORNERS, featureParams.qualityLevel,
		featureParams.minDistance, mask,
		featureParams.blockSize, featureParams.useHarrisDetector);

	auto &cornerParams = params_.cornerParams;
	auto &opticalFlowParams = params_.opticalFlowParams;

	cornerSubPix(
		frameGray, newPoints,
		cornerParams.subPixWinSize, cornerParams.zeroZone,
		opticalFlowParams.criteria);

	featuresAdded_ = newPoints.size();

	for (auto &newPointPosition : newPoints) {
		featurePoints_.push_back( FeaturePoint() );
		featurePoints_.back().addPos(move(newPointPosition), frameDeltaTime_);
	}
}



Mat Scene::MaskExistingFeatures(const Size &frameSize) const 
{
	Mat mask = Mat::zeros(frameSize, CV_8UC1); // type of mask is CV_8U
	mask.setTo(Scalar(255));

	auto &maskParams = params_.maskParams;

	for (const FeaturePoint &point : featurePoints_) {
		circle(
			mask, point.currentPos(),
			maskParams.radius, maskParams.color, maskParams.thickness);
	}

	return mask;
}

//#include <fstream>

void Scene::TrackFeatures(InputArray prevGray, InputArray frameGray, InputOutputArray visualFrame) 
{
	if (featurePoints_.empty())
		return;

	FeaturesPositions prevPoitions = GetLastFeaturesPositions();
	FeaturesPositions newPositions;
	vector<uchar> status;
	vector<float> err;

	auto &opticalFlowParams = params_.opticalFlowParams;

	calcOpticalFlowPyrLK(
		prevGray, frameGray,
		prevPoitions, newPositions,
		status, err,
		opticalFlowParams.winSize, opticalFlowParams.maxLevel,
		opticalFlowParams.criteria,
		opticalFlowParams.flags, opticalFlowParams.minEigThreshold);

	// back-track
	FeaturesPositions reversePositions;
	calcOpticalFlowPyrLK(
		frameGray, prevGray,
		newPositions, reversePositions,
		status, err,
		opticalFlowParams.winSize, opticalFlowParams.maxLevel,
		opticalFlowParams.criteria,
		opticalFlowParams.flags, opticalFlowParams.minEigThreshold);

	vector<bool> goodTracks = GoodTrackedPoints(prevPoitions, reversePositions);

	size_t addedFeaturesIdx = newPositions.size() - featuresAdded_;

	for (size_t i = 0; i < newPositions.size(); i++) {
		auto &featurePoint = featurePoints_[i];

		if (!goodTracks[i] && featurePoint.active) {
			featurePoint.active = false;
			DrawFeature(visualFrame, featurePoint, FeatureState_Bad);

			/*ofstream file("out.txt");
			file << featurePoint.track;
			file.close();*/

			continue;
		}

		featurePoint.addPos(newPositions[i], frameDeltaTime_);

		if (i < addedFeaturesIdx)
			DrawFeature(visualFrame, featurePoint, FeatureState_Normal);
		else
			DrawFeature(visualFrame, featurePoint, FeatureState_New);
	}
}



Scene::FeaturesPositions Scene::GetLastFeaturesPositions() const
{
	vector< FeaturePoint::Point_t > lastFeaturePositions;
	lastFeaturePositions.reserve( featurePoints_.size() );
	for (const auto &point : featurePoints_) {
		lastFeaturePositions.push_back( point.currentPos() );
	}

	return lastFeaturePositions;
}



vector<bool> Scene::GoodTrackedPoints(const FeaturesPositions &original, const FeaturesPositions &reverse) const
{
	vector<bool> goodTracks(original.size());

	for (size_t i = 0; i < original.size(); i++) {
		float x_val = abs(original[i].x - reverse[i].x);
		float y_val = abs(original[i].y - reverse[i].y);

		goodTracks[i] = (max(x_val, y_val) < 1.0);
	}

	return goodTracks;
}



size_t Scene::GetActiveFecturePoints() const
{
	size_t count = 0;
	for (const auto &featurePoint : featurePoints_) {
		if (featurePoint.active)
			count++;
	}
	return count;
}



void Scene::DrawFeature(InputOutputArray visualFrame, const FeaturePoint& featurePoint, FeatureStates featureState) const
{
	Scalar color;

	switch (featureState)
	{
	case FeatureState_Normal:
		color = Scalar(0, 255, 0);
		break;

	case FeatureState_New:
		color = Scalar(255, 0, 0);
		break;

	case FeatureState_Bad:
		color = Scalar(0, 0, 255);
		break;
	}

	DrawPoint(visualFrame, featurePoint, color);
	DrawTrack(visualFrame, featurePoint, color);
	DrawVelocity(visualFrame, featurePoint, Scalar(127, 127, 0));
}



void Scene::DrawPoint(InputOutputArray visualFrame, const FeaturePoint& featurePoint, const Scalar &color) const
{
	circle(visualFrame, featurePoint.currentPos(), 3, color, -1);
}



void Scene::DrawTrack(InputOutputArray visualFrame, const FeaturePoint& featurePoint, const Scalar &color) const
{
	auto first = featurePoint.track.begin();
	auto last = featurePoint.track.end();

	if (featurePoint.track.size() > trackLength_)
		first = last - trackLength_;

	Mat line = Mat(vector<Point2f>(first, last), true);
	line.convertTo(line, CV_32S);
	polylines(visualFrame, line, false, color);
}



void Scene::DrawVelocity(InputOutputArray visualFrame, const FeaturePoint& featurePoint, const Scalar& color) const
{
	if (featurePoint.velocities.size() >= 3) {
		auto currentVeloc = featurePoint.currentVeloc();

		FeaturePoint::Point_t from = featurePoint.currentPos();
		FeaturePoint::Point_t to = currentVeloc + from;
		arrowedLine(visualFrame, from, to, color);

		stringstream stream;
		stream << int(norm(currentVeloc));
		Point textPos = Point(from) + Point(5, 0);
		putText(visualFrame, stream.str(), textPos, FONT_HERSHEY_SIMPLEX, 0.3, Scalar(255, 255, 255), 0, CV_AA);
	}
}



void Scene::ShowFeaturesInfo(InputOutputArray visualFrame) const
{
	stringstream stream;
	stream << "track total: " << featurePoints_.size();
	DrawText(visualFrame, stream.str(), Point(20, 20));

	stream.str("");
	stream << "active tracks: " << GetActiveFecturePoints();
	DrawText(visualFrame, stream.str(), Point(20, 40));
}



void Scene::DrawText(InputOutputArray visualFrame, const string& str, const Point& pos) const
{
	putText(visualFrame, str, pos, CV_FONT_HERSHEY_PLAIN, 1.0, Scalar(255, 255, 255), 1, LINE_4);
}