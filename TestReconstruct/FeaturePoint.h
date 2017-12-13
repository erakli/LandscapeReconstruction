#pragma once

#include <deque>

#include "SimpleMappingCore.h"

namespace SimpleMapping 
{

	class FeaturePoint 
	{
	public:
		typedef Point2f				Point_t;
		typedef vector< Point_t >	PointTrack;

		FeaturePoint();
		~FeaturePoint();

		void addPos(const Point_t& position, double dt);
		Point_t currentPos() const;
		Point_t currentVeloc() const;

		PointTrack track;
		PointTrack velocities;

		Point2d velocitiesSum;

		bool active;
	};

}

