#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"	// cornerSubPix
#include "opencv2/video/tracking.hpp"	// calcOpticalFlowPyrLK

#include <deque>
#include <iostream>

#include "Scene.h"

using namespace cv;
using namespace std;

using namespace SimpleMapping;


// TODO: стоит ввести допущение, что направление движения заранее известно

// ----------------------------------------------------------------
// Appearance
// ----------------------------------------------------------------

const string WINDOW_NAME = "test_reconstruct";

void PrepareWindow(const Size2d &frameDims)
{
	double scaleCoeff = 0.5;
	Size windowDims(frameDims.width * scaleCoeff,
		frameDims.height * scaleCoeff);

	namedWindow(WINDOW_NAME, WINDOW_KEEPRATIO);
	resizeWindow(WINDOW_NAME, windowDims.width, windowDims.height);
	moveWindow(WINDOW_NAME, 0, 0);

#ifdef SHOW_MASK
	namedWindow("mask", WINDOW_KEEPRATIO);
	resizeWindow("mask", windowDims.width, windowDims.height);
	moveWindow("mask", windowDims.width, 0);
#endif
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

	const int DELAY = 5;

	Scene scene( cap.get(CV_CAP_PROP_FPS) );

	size_t detectInterval = 30;
	size_t frameIdx = 0;
	Mat prevGray;

	try {
		while (true) {
			Mat frame;
			cap >> frame;
			if (frame.empty())
				break;

			Mat frameGray;
			cvtColor(frame, frameGray, COLOR_BGR2GRAY);
			Mat visualFrame = frame;

			scene.TrackFeatures(prevGray, frameGray, visualFrame);
	
			if (frameIdx % detectInterval == 0) {
				scene.FindNewFeatures(frameGray);
			}

			scene.ShowFeaturesInfo(visualFrame);

			imshow(WINDOW_NAME, visualFrame);

			char c = static_cast<char>(waitKey(DELAY));
			if (c == 27)
				break;

			cv::swap(prevGray, frameGray);
			frameIdx++;
		}
	}
	catch (cv::Exception &e) {
		cerr << e.msg << endl;
		waitKey(5000);
	}

	return 0;
}