#include "Camera.h"

using namespace SimpleMapping;


const double MM_IN_M = 10.0 * 100.0;


Camera::Camera(double focal_mm, Point2i centerPoint, double pixelSize_mm, double veloc)
	: focal_m(focal_mm / MM_IN_M)
	, centerPoint(centerPoint)
	, pixelSize_m(pixelSize_mm / MM_IN_M)
	, veloc(veloc)
{
}

Camera::~Camera()
{
}
