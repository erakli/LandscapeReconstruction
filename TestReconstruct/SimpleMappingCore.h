#pragma once

#include <cassert>

#include <opencv2/core/mat.hpp>

namespace SimpleMapping 
{

	using namespace cv;
	using namespace std;


	typedef Point2d				Veloc_t;


	template<class T>
	Point_<T> Derivate(const Point_<T> &x1, const Point_<T> &x2, double delta) {
		return (x2 - x1) / delta;
	}


	template<class T>
	Point_<T> CentralDifferenceDerivate(const vector< Point_<T> > &f, size_t i, double h) {
		assert(f.size() >= 3);
		assert( (i > 0) && (i < f.size() - 1) );

		return Derivate(f[i - 1], f[i + 1], 2.0 * h);
	}

}