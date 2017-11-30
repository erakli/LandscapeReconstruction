#include <vector>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>	// ORB

using namespace cv;
using namespace std;

int main( int argc, char** argv )
{
    if( argc != 2 ) {
     cout <<" Usage: display_image ImageToLoadAndDisplay" << endl;
     return -1;
    }

	try {
		Mat image;
		image = imread(argv[1], IMREAD_GRAYSCALE); // Read the file
	
		if( image.empty() ) { // Check for invalid input
			cout << "Could not open or find the image" << endl ;
			return -1;
		}

		namedWindow("Display window", WINDOW_AUTOSIZE); // Create a window for display.
		imshow("Display window", image); // Show our image inside it.

		Ptr< FeatureDetector > detector = ORB::create();
		vector<KeyPoint> keyPoints;
		detector->detect(image, keyPoints);

		Mat descriptors;
		detector->compute(image, keyPoints, descriptors);

		Mat resultImage;
		drawKeypoints(image, keyPoints, resultImage, Scalar(0, 255, 0));

		namedWindow("Result image", WINDOW_AUTOSIZE); // Create a window for display.
		imshow("Result image", resultImage); // Show our image inside it.
	}
	catch (cv::Exception &e) {
		cout << e.msg << endl;
	}
	catch (exception &e) {
		cout << e.what() << endl;
	}

	waitKey(0); // Wait for a keystroke in the window

    return 0;
}