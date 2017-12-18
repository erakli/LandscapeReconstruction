#pragma once

//#include <deque>

#include "SimpleMappingCore.h"

namespace SimpleMapping 
{

	class FeaturePoint 
	{
	public:
		typedef Point2f				Point_t;
		typedef vector< Point_t >	PointTrack;

		typedef vector< Veloc_t >	PointVelocities;

		FeaturePoint();
		~FeaturePoint();

		void addPos(const Point_t& position, double dt);
		void evalVeloc(double dt);

		Point_t currentPos() const;
		Veloc_t currentVeloc() const;

		Point_t initialPos() const;
		Veloc_t meanVeloc() const;

		bool couldUseVeloc() const;

		PointTrack track;
		PointVelocities velocities;

		Veloc_t velocitySum;

		size_t MIN_VELOC_SIZE;

		bool active;
		bool bad;
	};

}

