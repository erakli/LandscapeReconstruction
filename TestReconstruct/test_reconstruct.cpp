#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"	// cornerSubPix
#include "opencv2/video/tracking.hpp"	// calcOpticalFlowPyrLK

#include <deque>
#include <iostream>

using namespace cv;
using namespace std;


// TODO: стоит ввести допущение, что направление движения заранее известно


const string WINDOW_NAME = "test_reconstruct";

const struct FeatureParams_ {
	int MAX_CORNERS = 500;
	double qualityLevel = 0.3;
	double minDistance = 7.0;
	int blockSize = 7;
	bool useHarrisDetector = false;
} featureParams;

const struct CornerParams_ {
	Size subPixWinSize = Size(10, 10);
	Size zeroZone = Size(-1, -1);
} cornerParams;

const struct OpticalFlowParams_ {
	Size winSize = Size(15, 15);
	int maxLevel = 2;
	TermCriteria criteria =
		TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 10, 0.03);
	int flags = 0;
	double minEigThreshold = 1e-4;
} opticalFlowParams;

const struct MaskParams_ {
	int radius = 15;
	Scalar color = Scalar(0);
	int thickness = -1;
} maskParams;

size_t trackLength = 10;
size_t detectInterval = 5;
size_t frameIdx = 0;

Mat prevGray;
typedef deque< Point2f >	PointTrack;
vector< PointTrack > pointsTracks;



void PrepareWindow(const Size2d &frameDims)
{
	double scaleCoeff = 0.5;
	Size windowDims(frameDims.width * scaleCoeff,
					frameDims.height * scaleCoeff);

	namedWindow(WINDOW_NAME, WINDOW_KEEPRATIO);
	resizeWindow(WINDOW_NAME, windowDims.width, windowDims.height);
}



void FindNewFeatures(InputArray frame, vector< PointTrack > &cornerPointsTracks) {
	Mat mask = Mat::zeros(frame.size(), CV_8UC1); // type of mask is CV_8U
	mask.setTo(Scalar(255));

	for (const PointTrack &track : cornerPointsTracks) {
		circle(
			mask, track.back(), 
			maskParams.radius, maskParams.color, maskParams.thickness);
	}

	vector< Point2f > newPoints;
	goodFeaturesToTrack(
		frame, newPoints,
		featureParams.MAX_CORNERS, featureParams.qualityLevel,
		featureParams.minDistance, mask,
		featureParams.blockSize, featureParams.useHarrisDetector);

	cornerSubPix(
		frame, newPoints,
		cornerParams.subPixWinSize, cornerParams.zeroZone,
		opticalFlowParams.criteria);

	for (auto &newPointPosition : newPoints) {
		cornerPointsTracks.push_back( PointTrack() );
		cornerPointsTracks.back().push_back( move(newPointPosition) );
	}
}



vector< Point2f > GetLastPointPositions(const vector< PointTrack > &pointsTracks) {
	vector< Point2f > lastPointPoitions;
	lastPointPoitions.reserve( pointsTracks.size() );
	for (const auto &pointTrack : pointsTracks)
		lastPointPoitions.push_back( pointTrack.back() );

	return lastPointPoitions;
}


void TrackFeatures(InputArray frameGray, vector< PointTrack > &cornerPointsTracks, InputOutputArray visualFrame) {
	if (cornerPointsTracks.empty())
		return;

	vector< Point2f > lastPointsPoitions = GetLastPointPositions(cornerPointsTracks);
	vector< Point2f > newPoints;
	vector<uchar> status;
	vector<float> err;

	calcOpticalFlowPyrLK(
		prevGray, frameGray,
		lastPointsPoitions, newPoints,
		status, err,
		opticalFlowParams.winSize, opticalFlowParams.maxLevel,
		opticalFlowParams.criteria,
		opticalFlowParams.flags, opticalFlowParams.minEigThreshold);

	// TODO: можно добавить обратную проверку

	vector< PointTrack > newTracks;
	newTracks.reserve( cornerPointsTracks.size() );
	for (size_t i = 0; i < newPoints.size(); i++) {
		if (!status[i])
			continue;

		const Point2f &point = newPoints[i];
		PointTrack track = move(cornerPointsTracks[i]);
		track.push_back(point);
		if (track.size() > trackLength)
			track.pop_front();

		newTracks.push_back( move(track) );

		circle(visualFrame, point, 3, Scalar(0, 255, 0), -1);
	}
	
	cornerPointsTracks = move(newTracks);
	//polylines(visualFrame, cornerPointsTracks, false, Scalar(0, 255, 0));
}



int main(int argc, char** argv)
{
	if (argc != 2) {
		cout << " Usage: test_reconstruct videoFilePath" << endl;
		return -1;
	}
	
	VideoCapture cap(argv[1]);

	if ( !cap.isOpened() ) {
		cout << "Could not initialize capturing...\n";
		return 0;
	}

	Size2d frameDims(	cap.get(CV_CAP_PROP_FRAME_WIDTH),
						cap.get(CV_CAP_PROP_FRAME_HEIGHT) );

	PrepareWindow(frameDims);

	while (true) {
		Mat frame;
		cap >> frame;
		if (frame.empty())
			break;

		Mat frameGray;
		cvtColor(frame, frameGray, COLOR_BGR2GRAY);
		Mat visualFrame = frame;

		TrackFeatures(frameGray, pointsTracks, visualFrame);
	
		if (frameIdx % detectInterval == 0) {
			FindNewFeatures(frameGray, pointsTracks);
		}

		imshow(WINDOW_NAME, visualFrame);

		char c = static_cast<char>(waitKey(2));
		if (c == 27)
			break;

		cv::swap(prevGray, frameGray);
		frameIdx++;
	}

	return 0;
}