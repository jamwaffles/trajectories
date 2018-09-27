/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Tobias Kunz <tobias@gatech.edu>
 * Date: 05/2012
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 * Algorithm details and publications:
 * http://www.golems.org/node/1570
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "Path.hpp"
#include <fstream>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

using namespace std;
using namespace Eigen;


class LinearPathSegment : public PathSegment
{
public:
	LinearPathSegment(const Eigen::Vector3d &start, const Eigen::Vector3d &end) :
		PathSegment((end-start).norm()),
		start(start),
		end(end)
	{
	}

	Eigen::Vector3d getConfig(double s) const {
		s /= length;
		s = std::max(0.0, std::min(1.0, s));
		return (1.0 - s) * start + s * end;
	}

	Eigen::Vector3d getTangent(double /* s */) const {
		return (end - start) / length;
	}

	Eigen::Vector3d getCurvature(double /* s */) const {
		return Eigen::Vector3d::Zero(start.size());
	}

	list<double> getSwitchingPoints() const {
		return list<double>();
	}

	LinearPathSegment* clone() const {
		return new LinearPathSegment(*this);
	}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	Eigen::Vector3d start;
	Eigen::Vector3d end;
};


class CircularPathSegment : public PathSegment
{
public:
	CircularPathSegment(const Eigen::Vector3d &start, const Eigen::Vector3d &intersection, const Eigen::Vector3d &end, double maxDeviation) {
		if((intersection - start).norm() < 0.000001 || (end - intersection).norm() < 0.000001) {
			length = 0.0;
			radius = 1.0;
			center = intersection;
			x = Eigen::Vector3d::Zero(start.size());
			y = Eigen::Vector3d::Zero(start.size());
			return;
		}

		const Eigen::Vector3d startDirection = (intersection - start).normalized();
		const Eigen::Vector3d endDirection = (end - intersection).normalized();

		if((startDirection - endDirection).norm() < 0.000001) {
			length = 0.0;
			radius = 1.0;
			center = intersection;
			x = Eigen::Vector3d::Zero(start.size());
			y = Eigen::Vector3d::Zero(start.size());
			return;
		}

		// const double startDistance = (start - intersection).norm();
		// const double endDistance = (end - intersection).norm();

		double distance = std::min((start - intersection).norm(), (end - intersection).norm());
		const double angle = acos(startDirection.dot(endDirection));

		distance = std::min(distance, maxDeviation * sin(0.5 * angle) / (1.0 - cos(0.5 * angle)));  // enforce max deviation

		radius = distance / tan(0.5 * angle);
		length = angle * radius;

		center = intersection + (endDirection - startDirection).normalized() * radius / cos(0.5 * angle);

		x = (intersection - distance * startDirection - center).normalized();
		y = startDirection;

		// IOFormat CommaInitFmt(StreamPrecision, DontAlignCols, ", ", ", ", "", "", " << ", ";");

		// std::cout << " X " << x << std::endl;
		// std::cout << " Y " << y << std::endl;
	}

	Eigen::Vector3d getConfig(double s) const {
		const double angle = s / radius;
		return center + radius * (x * cos(angle) + y * sin(angle));
	}

	Eigen::Vector3d getTangent(double s) const {
		const double angle = s / radius;
		return - x * sin(angle) + y * cos(angle);
	}

	Eigen::Vector3d getCurvature(double s) const {
		const double angle = s / radius;
		return - 1.0 / radius * (x * cos(angle) + y * sin(angle));
	}

	list<double> getSwitchingPoints() const {
		list<double> switchingPoints;
		const double dim = x.size();
		for(unsigned int i = 0; i < dim; i++) {
			double switchingAngle = atan2(y[i], x[i]);
			if(switchingAngle < 0.0) {
				switchingAngle += M_PI;
			}
			const double switchingPoint = switchingAngle * radius;
			if(switchingPoint < length) {
				switchingPoints.push_back(switchingPoint);
			}
		}
		switchingPoints.sort();

		// std::cout << "---" << std::endl;
		// for(std::list<double>::iterator p = switchingPoints.begin(); p != switchingPoints.end(); p++) {
		// 	std::cout << "POINT " << *p << std::endl;
		// }
		// std::cout << "---" << std::endl;

		return switchingPoints;
	}

	CircularPathSegment* clone() const {
		return new CircularPathSegment(*this);
	}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	double radius;
	Eigen::Vector3d center;
	Eigen::Vector3d x;
	Eigen::Vector3d y;
};



Path::Path(const list<Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &path, double maxDeviation) :
	length(0.0)
{
	if(path.size() < 2)
		return;
	list<Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >::const_iterator config1 = path.begin();
	list<Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >::const_iterator config2 = config1;
	config2++;
	list<Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >::const_iterator config3;
	Vector3d startConfig = *config1;
	while(config2 != path.end()) {
		config3 = config2;
		config3++;
		if(maxDeviation > 0.0 && config3 != path.end()) {
			CircularPathSegment* blendSegment = new CircularPathSegment(0.5 * (*config1 + *config2), *config2, 0.5 * (*config2 + *config3), maxDeviation);
			Vector3d endConfig = blendSegment->getConfig(0.0);
			if((endConfig - startConfig).norm() > 0.000001) {
				pathSegments.push_back(new LinearPathSegment(startConfig, endConfig));
			}
			pathSegments.push_back(blendSegment);

			startConfig = blendSegment->getConfig(blendSegment->getLength());
		}
		else {
			pathSegments.push_back(new LinearPathSegment(startConfig, *config2));
			startConfig = *config2;
		}
		config1 = config2;
		config2++;
	}

	// create list of switching point candidates, calculate total path length and absolute positions of path segments
	for(list<PathSegment*>::iterator segment = pathSegments.begin(); segment != pathSegments.end(); segment++) {
		(*segment)->position = length;
		list<double> localSwitchingPoints = (*segment)->getSwitchingPoints();

		// std::cout << "LEN " << length << std::endl;

		// for(std::list<double>::iterator i = localSwitchingPoints.begin(); i != localSwitchingPoints.end(); i++) {
		// 	std::cout << "Switching point " << *i << std::endl;
		// }

		// std::cout << "---" << std::endl;

		for(list<double>::const_iterator point = localSwitchingPoints.begin(); point != localSwitchingPoints.end(); point++) {
			// Add switching point along segment, offset by current length, where discontinuous = false
			switchingPoints.push_back(make_pair(length + *point, false));
		}
		length += (*segment)->getLength();
		// Remove switching points that are at or beyond the end of the most recent segment
		while(!switchingPoints.empty() && switchingPoints.back().first >= length)
			switchingPoints.pop_back();

		// Add switching point at end of segment that has discontinuous = true
		switchingPoints.push_back(make_pair(length, true));
	}
	switchingPoints.pop_back();

	// for(std::list<std::pair<double, bool> >::iterator i = switchingPoints.begin(); i != switchingPoints.end(); i++) {
	// 	std::cout << "Switching point " << (*i).first << " " << (*i).second << std::endl;
	// }
}

Path::Path(const Path &path) :
	length(path.length),
	switchingPoints(path.switchingPoints)
{
	for(list<PathSegment*>::const_iterator it = path.pathSegments.begin(); it != path.pathSegments.end(); it++) {
		pathSegments.push_back((*it)->clone());
	}
}

Path::~Path() {
	for(list<PathSegment*>::iterator it = pathSegments.begin(); it != pathSegments.end(); it++) {
		delete *it;
	}
}

double Path::getLength() const {
	return length;
}

PathSegment* Path::getPathSegment(double &s) const {
	list<PathSegment*>::const_iterator it = pathSegments.begin();
	list<PathSegment*>::const_iterator next = pathSegments.begin();
	std::advance(next, 1);

	unsigned int i = 1;

	// while(next != pathSegments.end() && s >= (*next)->position) {
	while(i < pathSegments.size() && s >= (*next)->position) {
		std::advance(it, 1);
		std::advance(next, 1);

		i++;
	}
	s -= (*it)->position;
	return *it;
}

Vector3d Path::getConfig(double s) const {
	const PathSegment* pathSegment = getPathSegment(s);
	return pathSegment->getConfig(s);
}

Vector3d Path::getTangent(double s) const {
	const PathSegment* pathSegment = getPathSegment(s);

	return pathSegment->getTangent(s);
}

Vector3d Path::getCurvature(double s) const {
	const PathSegment* pathSegment = getPathSegment(s);
	return pathSegment->getCurvature(s);
}

double Path::getNextSwitchingPoint(double s, bool &discontinuity) const {
	list<pair<double, bool> >::const_iterator it = switchingPoints.begin();
	while(it != switchingPoints.end() && it->first <= s) {
		it++;
	}
	if(it == switchingPoints.end()) {
		discontinuity = true;
		return length;
	}
	else {
		discontinuity = it->second;
		return it->first;
	}
}

list<pair<double, bool> > Path::getSwitchingPoints() const {
	return switchingPoints;
}
