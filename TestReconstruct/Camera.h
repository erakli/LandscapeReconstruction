#pragma once

#include "SimpleMappingCore.h"

namespace SimpleMapping
{

	class Camera
	{
	public:
		Camera(double focal_mm, Point2i centerPoint, double pixelSize_mm, double veloc);
		~Camera();

		double focal_m;			// m
		Point2i centerPoint;	// pixels
		double pixelSize_m;		// m

		// считаем скорость постоянной (описывается одним вектором,
		// сонаправленном среднему вектору скорости всех точек)
		double veloc;	
	};

}
