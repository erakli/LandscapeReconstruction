#include "FeaturePoint.h"

// TODO: нужно усреднение вектора скорости

using namespace SimpleMapping;


FeaturePoint::FeaturePoint()
	: MIN_VELOC_SIZE(3)
	, active(true)
	, bad(false)
{
}


FeaturePoint::~FeaturePoint()
{
}



void FeaturePoint::addPos(const Point_t& position, double dt)
{
	track.push_back(position);
	if (active && !bad) {
		evalVeloc(dt);
	}
}



void FeaturePoint::evalVeloc(double dt)
{
	velocities.push_back(Veloc_t(0, 0));
	if (track.size() >= MIN_VELOC_SIZE) {
		size_t i = track.size() - (MIN_VELOC_SIZE - 1);
		velocities[i] = CentralDifferenceDerivate(track, i, dt);
		velocities[i + 1] = velocities[i];

		velocitySum += velocities[i];
	}
}



FeaturePoint::Point_t FeaturePoint::currentPos() const
{
	return track.back();
}



Veloc_t FeaturePoint::currentVeloc() const
{
	return velocities.back();
}



FeaturePoint::Point_t FeaturePoint::initialPos() const
{
	return track.front();
}


// без проверки
Veloc_t FeaturePoint::meanVeloc() const
{
	// первую и последнюю скорости не учитываем
	return velocitySum / double(velocities.size() - 2);
}



bool FeaturePoint::couldUseVeloc() const
{
	return velocities.size() > MIN_VELOC_SIZE;
}
