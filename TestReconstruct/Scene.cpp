#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp> // calcOpticalFlowPyrLK
#include <opencv2/imgcodecs.hpp> // imwrite

#include <windows.h>

// так как у нас есть уже такая функция, уберём макрос
#undef DrawText

#include <fstream>
#include <iomanip>

#ifdef _DEBUG
#include <iostream>
#endif

#include "Scene.h"

using namespace SimpleMapping;


Scene::Scene(const Camera &camera, double frameRate)
	: frameIdx_(0)
	, lastKeyFrame_(0)
	, featuresAdded_(0)
	, camera_(camera)
	, trackLength_( size_t( round(frameRate) ) )
	, frameDeltaTime_(1.0 / frameRate)
{
	CreateDirectory("output", NULL);
}

Scene::~Scene()
{
}



Mat Scene::ProcessNewFrame(const Mat &inputFrame)
{
	cvtColor(inputFrame, frameGray_, COLOR_BGR2GRAY);
	visualFrame_ = inputFrame;

	TrackFeatures();

	if ( NeedToFindNewFeatures() ) {
		if (frameIdx_ != 0) {
			SaveWorldPoints();
		}
		FindNewFeatures();
		SaveFrame();
	}

	ShowFeaturesInfo();

	cv::swap(prevGray_, frameGray_);
	frameIdx_++;

	return visualFrame_;
}



void Scene::SaveWorldPoints()
{
	vector< Point3d > worldPoints = FindWorldPoints();
	SaveWorldPoints(worldPoints);
}


bool Scene::NeedToFindNewFeatures() const
{
	//return (frameIdx_ % params_.detectInterval == 0);
	return (GetActiveFeaturePointsCount() == 0);
}



void Scene::SaveFrame()
{
	lastKeyFrame_ = frameIdx_;

	Mat keyFrame;
	visualFrame_.copyTo(keyFrame);

	auto currentFeaturePoints = GetLastAddedFeaturePoints();

	for (const FeaturePoint &feature : currentFeaturePoints) {
		circle(keyFrame, feature.initialPos(), 3, Scalar(0, 255, 0), -1);
	}

	imwrite("output/key_frame_" + to_string(frameIdx_) + ".png", keyFrame);
}



void Scene::SaveWorldPoints(const vector< Point3d > &worldPoints)
{
	ofstream file("output/key_frame_heights_" + to_string(lastKeyFrame_) + ".txt");
	file << setprecision(10);
	for (auto &point : worldPoints) {
		file << point.x << ", " << point.y << ", " << point.z << endl;
	}
	file.close();
}



vector< Point3d > Scene::FindWorldPoints()
{
	// мы предполагаем, что во всех алгоритмах одинаковый порядок всех элементовы
	vector< FeaturePointRef > goodFeaturePoints = GetLastGoodFeaturePoints();
	vector< Point3d > worldPoints(goodFeaturePoints.size());

	double maxDistance = 0.0;

	for (size_t i = 0; i < goodFeaturePoints.size(); i++) {
		worldPoints[i] = EvalWorldVec(goodFeaturePoints[i].get());
		if (maxDistance < worldPoints[i].z)
			maxDistance = worldPoints[i].z;
	}

	// вычислим таким образом высоты
	for_each(
		worldPoints.begin(),
		worldPoints.end(),
		[maxDistance](Point3d &point){ point.z = maxDistance - point.z; });

	return worldPoints;
}



Point3d Scene::EvalWorldVec(const FeaturePoint &featurePoint) const
{
	FeaturePoint::Point_t imagePlanarPos_m = 
		camera_.pixelSize_m *
		(featurePoint.initialPos() - FeaturePoint::Point_t(camera_.centerPoint));

	Point3d imageVec(imagePlanarPos_m.x, imagePlanarPos_m.y, camera_.focal_m);
	Veloc_t imagePlanarVeloc_m = featurePoint.meanVeloc() * camera_.pixelSize_m;

	return (camera_.veloc / norm(imagePlanarVeloc_m)) * imageVec;
}



void Scene::FindNewFeatures() 
{
	Mat mask = MaskExistingFeatures(frameGray_.size());

	auto &featureParams = params_.featureParams;

	FeaturesPositions newPositions;
	goodFeaturesToTrack(
		frameGray_, newPositions,
		featureParams.MAX_CORNERS, featureParams.qualityLevel,
		featureParams.minDistance, mask,
		featureParams.blockSize, featureParams.useHarrisDetector);

	auto &cornerParams = params_.cornerParams;
	auto &opticalFlowParams = params_.opticalFlowParams;

	cornerSubPix(
		frameGray_, newPositions,
		cornerParams.subPixWinSize, cornerParams.zeroZone,
		opticalFlowParams.criteria);

	featuresAdded_ = newPositions.size();

	for (auto &newPointPosition : newPositions) {
		featurePoints_.push_back( FeaturePoint() );
		featurePoints_.back().addPos(move(newPointPosition), frameDeltaTime_);
	}
}



Mat Scene::MaskExistingFeatures(const Size &frameSize) const 
{
	Mat mask = Mat::zeros(frameSize, CV_8UC1); // type of mask is CV_8U
	mask.setTo(Scalar(255));

	auto &maskParams = params_.maskParams;

	// маскируем все (даже плохие) активные точки
	for (const FeaturePoint &featurePoint : featurePoints_) {
		if (featurePoint.active) {
			circle(
				mask, featurePoint.currentPos(),
				maskParams.radius, maskParams.color, maskParams.thickness);
		}
	}

	return mask;
}



void Scene::TrackFeatures() 
{
	if (featurePoints_.empty())
		return;

	FeaturesPositions prevPoitions = GetLastFeaturesPositions();
	FeaturesPositions newPositions;
	vector<uchar> status;
	vector<float> err;

	auto &opticalFlowParams = params_.opticalFlowParams;

	calcOpticalFlowPyrLK(
		prevGray_, frameGray_,
		prevPoitions, newPositions,
		status, err,
		opticalFlowParams.winSize, opticalFlowParams.maxLevel,
		opticalFlowParams.criteria,
		opticalFlowParams.flags, opticalFlowParams.minEigThreshold);

	// back-track
	FeaturesPositions reversePositions;
	calcOpticalFlowPyrLK(
		frameGray_, prevGray_,
		newPositions, reversePositions,
		status, err,
		opticalFlowParams.winSize, opticalFlowParams.maxLevel,
		opticalFlowParams.criteria,
		opticalFlowParams.flags, opticalFlowParams.minEigThreshold);

	vector<bool> goodTracks = GoodTrackedPoints(prevPoitions, reversePositions);

	size_t addedFeaturesIdx = newPositions.size() - featuresAdded_;

	for (size_t i = 0; i < newPositions.size(); i++) {
		FeaturePoint &featurePoint = featurePoints_[i];

		if (!goodTracks[i]) {
			if (featurePoint.active) {
				featurePoint.active = false;
				DrawFeature(featurePoint, FeatureState_Inactive);
			}

			continue;
		}

		featurePoint.addPos(newPositions[i], frameDeltaTime_);

		bool isNew = (i < addedFeaturesIdx);
		DrawActiveFeature(featurePoint, isNew);
	}

	MarkBadFeatures();
}



Scene::FeaturesPositions Scene::GetLastFeaturesPositions() const
{
	vector< FeaturePoint::Point_t > lastFeaturePositions;
	lastFeaturePositions.reserve( featurePoints_.size() );

	for (const FeaturePoint &featurePoint : featurePoints_) {
		lastFeaturePositions.push_back(featurePoint.currentPos());
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



vector<Scene::FeaturePointRef> Scene::GetLastAddedFeaturePoints()
{
	size_t addedFeaturesIdx = featurePoints_.size() - featuresAdded_;

	vector< FeaturePointRef > lastAddedFeaturePoints;
	lastAddedFeaturePoints.reserve(featuresAdded_);

	for (size_t i = addedFeaturesIdx; i < featurePoints_.size(); i++) {
		lastAddedFeaturePoints.push_back(featurePoints_[i]);
	}
	return lastAddedFeaturePoints;
}



// берём из последних добавленных точек
vector<Scene::FeaturePointRef> Scene::GetLastGoodFeaturePoints()
{
	size_t addedFeaturesIdx = featurePoints_.size() - featuresAdded_;

	vector< FeaturePointRef > goodFeaturePoints;
	goodFeaturePoints.reserve(featuresAdded_);

	for (size_t i = addedFeaturesIdx; i < featurePoints_.size(); i++) {
		FeaturePoint &featurePoint = featurePoints_[i];
		if (!featurePoint.bad && featurePoint.couldUseVeloc())
			goodFeaturePoints.push_back(featurePoint);
	}
	return goodFeaturePoints;
}



vector< Scene::FeaturePointRef > Scene::GetActiveFeaturePoints()
{	
	vector< FeaturePointRef > activeFeaturePoints;
	activeFeaturePoints.reserve(featurePoints_.size());
	for (FeaturePoint &featurePoint : featurePoints_) {
		if (featurePoint.active && featurePoint.couldUseVeloc())
			activeFeaturePoints.push_back(featurePoint);
	}
	return activeFeaturePoints;
}


size_t Scene::GetActiveFeaturePointsCount() const
{
	size_t count = 0;
	for (const FeaturePoint &featurePoint : featurePoints_) {
		if (featurePoint.active)
			count++;
	}
	return count;
}



#if defined(_DEBUG) && defined(BAD_COUNT_OUTPUT)
size_t badCount = 0;
#endif

bool Scene::MarkBadFeatures()
{
	vector< FeaturePointRef > activeFeaturePoints = GetActiveFeaturePoints();
	EvalMeanVeloc(activeFeaturePoints);

	bool hasBadFeatures = false;

#if defined(_DEBUG) && defined(BAD_COUNT_OUTPUT)
	badCount = 0;
#endif

	for (auto &activeFeatureRef : activeFeaturePoints) {
		FeaturePoint &activeFeature = activeFeatureRef.get();
		
		if (!activeFeature.couldUseVeloc())
			continue;

		if ( CheckFeatureBad(activeFeature.currentVeloc()) ) {
			activeFeature.bad = true;
#if defined(_DEBUG) && defined(BAD_COUNT_OUTPUT)
			badCount++;
#endif
			if (!hasBadFeatures)
				hasBadFeatures = true;
		}
	}

#if defined(_DEBUG) && defined(BAD_COUNT_OUTPUT)
	if (badCount > 0)
		cout << "MarkBadFeatures::badCount = " << badCount << endl;
#endif

	return hasBadFeatures;
}



bool Scene::CheckFeatureBad(const Veloc_t& featureVeloc) const
{
	double featureVelocNorm = norm(featureVeloc);
	double cos_theta = meanVeloc_.vec.dot(featureVeloc) / (meanVeloc_.norm * featureVelocNorm);
	double theta = acos(cos_theta);

#if defined(_DEBUG) && defined(BAD_COUNT_OUTPUT)
	if (badCount > 0)
		cout << "CheckFeatureBad::theta = " << theta * 180.0 / CV_PI << ", eps = " << abs(featureVelocNorm - meanVeloc_.norm) << endl;
#endif

	return (
		(abs(theta) > params_.THETA_MAX) || 
		(abs(featureVelocNorm - meanVeloc_.norm) > meanVeloc_.maxEps));
}



void Scene::EvalMeanVeloc(const vector< FeaturePointRef > &activeFeaturePoints)
{
	Veloc_t meanVeloc(0.0, 0.0);
	for (const auto &featurePointRef : activeFeaturePoints) {
		meanVeloc += featurePointRef.get().currentVeloc();
	}
	
	meanVeloc_.vec = meanVeloc / double(activeFeaturePoints.size());
	meanVeloc_.norm = norm(meanVeloc_.vec);
	meanVeloc_.maxEps = meanVeloc_.norm * params_.NORM_EPS_COEFF;
}



void Scene::ShowFeaturesInfo() const
{
	stringstream stream;
	stream << "track total: " << featurePoints_.size();
	DrawText(stream.str(), Point(20, 20));

	stream.str("");
	stream << "active tracks: " << GetActiveFeaturePointsCount();
	DrawText(stream.str(), Point(20, 40));

	stream.str("");
	stream <<
		"meanVeloc_: " << meanVeloc_.vec <<
		", norm: " << meanVeloc_.norm <<
		", maxEps:" << meanVeloc_.maxEps;
	DrawText(stream.str(), Point(20, 60));
}



void Scene::DrawActiveFeature(const FeaturePoint& featurePoint, bool isNew) const
{
	if (!featurePoint.bad) {
		if (isNew)
			DrawFeature(featurePoint, FeatureState_Normal);
		else
			DrawFeature(featurePoint, FeatureState_New);
	}
	else
		DrawFeature(featurePoint, FeatureState_Bad);
}



void Scene::DrawFeature(const FeaturePoint& featurePoint, FeatureStates featureState) const
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

	case FeatureState_Inactive:
		color = Scalar(255, 255, 0);
		break;
	}

	DrawPoint(featurePoint, color);
	DrawTrack(featurePoint, color);
	//DrawVelocity(featurePoint, Scalar(127, 127, 255));
}



void Scene::DrawPoint(const FeaturePoint& featurePoint, const Scalar &color) const
{
	circle(visualFrame_, featurePoint.currentPos(), 3, color, -1);
}



void Scene::DrawTrack(const FeaturePoint& featurePoint, const Scalar &color) const
{
	auto first = featurePoint.track.begin();
	auto last = featurePoint.track.end();

	if (featurePoint.track.size() > trackLength_)
		first = last - trackLength_;

	Mat line = Mat(vector<Point2f>(first, last), true);
	line.convertTo(line, CV_32S);
	polylines(visualFrame_, line, false, color);
}



void Scene::DrawVelocity(const FeaturePoint& featurePoint, const Scalar& color) const
{
	auto currentVeloc = featurePoint.currentVeloc();

	FeaturePoint::Point_t from = featurePoint.currentPos();
	Veloc_t to = currentVeloc + Veloc_t(from);
	arrowedLine(visualFrame_, from, to, color);

	stringstream stream;
	stream << int(norm(currentVeloc));
	Point textPos = Point(from) + Point(5, 0);
	putText(visualFrame_, stream.str(), textPos, FONT_HERSHEY_SIMPLEX, 0.3, Scalar(255, 255, 255), 0, CV_AA);
}



void Scene::DrawText(const string& str, const Point& pos) const
{
	putText(visualFrame_, str, pos, CV_FONT_HERSHEY_PLAIN, 1.0, Scalar(255, 255, 255), 1, LINE_4);
}
