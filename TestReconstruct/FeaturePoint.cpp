#include "FeaturePoint.h"

// TODO: нужно усреднение вектора скорости

using namespace SimpleMapping;


FeaturePoint::FeaturePoint()
	: velocitiesSum(0.0)
	, active(true)
{
}


FeaturePoint::~FeaturePoint()
{
}



void FeaturePoint::addPos(const Point_t& position, double dt)
{
	track.push_back(position);
	velocities.push_back(Point_t(0, 0));
	if (track.size() >= 3) {
		size_t i = track.size() - 2;
		velocities[i] = CentralDifferenceDerivate(track, i, dt);
		velocities[i + 1] = velocities[i];
	}
}



FeaturePoint::Point_t FeaturePoint::currentPos() const
{
	return track.back();
}



FeaturePoint::Point_t FeaturePoint::currentVeloc() const
{
	return velocities.back();
}
