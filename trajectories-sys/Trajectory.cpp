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

#include "Trajectory.hpp"
#include <limits>
#include <iostream>
#include <fstream>

using namespace Eigen;
using namespace std;

const double Trajectory::eps = 0.000001;

// static double squared(double d) {
// 	return d * d;
// }f

Trajectory::Trajectory(const Path &path, const Vector3d &maxVelocity, const Vector3d &maxAcceleration, double timeStep) :
	path(path),
	maxVelocity(maxVelocity),
	maxAcceleration(maxAcceleration),
	n(maxVelocity.size()),
	valid(true),
	timeStep(timeStep),
	cachedTime(numeric_limits<double>::max())
{
	trajectory.push_back(TrajectoryStep(0.0, 0.0));
	double afterAcceleration = getMinMaxPathAcceleration(0.0, 0.0, true);
	while(valid && !integrateForward(trajectory, afterAcceleration) && valid) {
		double beforeAcceleration;
		TrajectoryStep switchingPoint;

		// COMP
		std::cout<<"CPP integ_fwd_end_step (pos;vel),"<<trajectory.back().pathPos<<","<<trajectory.back().pathVel<< std::endl;

		// Break if we've reached the end of the path
		if(getNextSwitchingPoint(trajectory.back().pathPos, switchingPoint, beforeAcceleration, afterAcceleration)) {
			break;
		}
		// COMP
		std::cout<<"CPP forward_sw_point (pos;vel;beforeAcceleration;afterAcceleration),"<<switchingPoint.pathPos<<","<<switchingPoint.pathVel<<","<<beforeAcceleration<<","<<afterAcceleration<<","<<std::endl;
		integrateBackward(trajectory, switchingPoint.pathPos, switchingPoint.pathVel, beforeAcceleration);
	}

	if(valid) {
		double beforeAcceleration = getMinMaxPathAcceleration(path.getLength(), 0.0, false);
		// std::cout << "beforeAcceleration " << beforeAcceleration << " path len " << path.getLength() << std::endl;
		integrateBackward(trajectory, path.getLength(), 0.0, beforeAcceleration);
	}

	if(valid) {
		// calculate timing
		list<TrajectoryStep>::iterator previous = trajectory.begin();
		list<TrajectoryStep>::iterator it = previous;
		it->time = 0.0;
		it++;
		while(it != trajectory.end()) {
			it->time = previous->time + (it->pathPos - previous->pathPos) / ((it->pathVel + previous->pathVel) / 2.0);

			previous++;
			it++;
		}

		// list<std::pair<double, bool> > thing = path.getSwitchingPoints();
		// list<std::pair<double, bool> >::iterator ass = thing.begin();
		// while(ass != thing.end()) {
		// 	std::cout.precision(std::numeric_limits<double>::digits10);
		// 	std::cout << "Switching point " << ass->first << "," << ass->second << std::endl;
		// 	ass++;
		// }

		// list<TrajectoryStep > thing2 = trajectory;
		// list<TrajectoryStep >::iterator ass2 = thing2.begin();
		// while(ass2 != thing2.end()) {
		// 	std::cout.precision(std::numeric_limits<double>::digits10);
		// 	std::cout << "Trajectory step position " << ass2->pathPos << ", velocity " << ass2->pathVel << ", time " << ass2->time << std::endl;
		// 	ass2++;
		// }
	}
}

Trajectory::~Trajectory(void) {
}

void Trajectory::outputPhasePlaneTrajectory() const {
	ofstream file1("maxVelocity.txt");
	const double stepSize = path.getLength() / 100000.0;
	for(double s = 0.0; s < path.getLength(); s += stepSize) {
		double maxVelocity = getAccelerationMaxPathVelocity(s);
		if(maxVelocity == numeric_limits<double>::infinity())
			maxVelocity = 10.0;
		file1 << s << "  " << maxVelocity << "  " << getVelocityMaxPathVelocity(s) << endl;
	}
	file1.close();

	ofstream file2("trajectory.txt");
	for(list<TrajectoryStep>::const_iterator it = trajectory.begin(); it != trajectory.end(); it++) {
		file2 << it->pathPos << "  " << it->pathVel << endl;
	}
	for(list<TrajectoryStep>::const_iterator it = endTrajectory.begin(); it != endTrajectory.end(); it++) {
		file2 << it->pathPos << "  " << it->pathVel << endl;
	}
	file2.close();
}

// returns true if end of path is reached.
bool Trajectory::getNextSwitchingPoint(double pathPos, TrajectoryStep &nextSwitchingPoint, double &beforeAcceleration, double &afterAcceleration) {
	// Acceleration switching point, velocity defaults to zero, filled in later
	TrajectoryStep accelerationSwitchingPoint(pathPos, 0.0);
	double accelerationBeforeAcceleration, accelerationAfterAcceleration;
	bool accelerationReachedEnd;

	// Find next switching point after current position that has a velocity below the velocity limit at that point
	do {
		accelerationReachedEnd = getNextAccelerationSwitchingPoint(accelerationSwitchingPoint.pathPos, accelerationSwitchingPoint, accelerationBeforeAcceleration, accelerationAfterAcceleration);
		// double test = getVelocityMaxPathVelocity(accelerationSwitchingPoint.pathPos);
	} while(!accelerationReachedEnd && accelerationSwitchingPoint.pathVel > getVelocityMaxPathVelocity(accelerationSwitchingPoint.pathPos));

	// COMP
	std::cout<<"CPP next_accel_sw_point (pos_along_path;sw_pos;sw_vel),"<<pathPos<<","<<accelerationSwitchingPoint.pathPos<<","<<accelerationSwitchingPoint.pathVel<<std::endl;

	// Velocity switching point, defaults to zero velocity
	TrajectoryStep velocitySwitchingPoint(pathPos, 0.0);
	double velocityBeforeAcceleration, velocityAfterAcceleration;
	bool velocityReachedEnd;
	do {
		velocityReachedEnd = getNextVelocitySwitchingPoint(velocitySwitchingPoint.pathPos, velocitySwitchingPoint, velocityBeforeAcceleration, velocityAfterAcceleration);
	} while(!velocityReachedEnd && velocitySwitchingPoint.pathPos <= accelerationSwitchingPoint.pathPos
		&& (velocitySwitchingPoint.pathVel > getAccelerationMaxPathVelocity(velocitySwitchingPoint.pathPos - eps)
		|| velocitySwitchingPoint.pathVel > getAccelerationMaxPathVelocity(velocitySwitchingPoint.pathPos + eps)));

	// COMP
	std::cout<<"CPP next_vel_sw_point (pos_along_path;sw_pos;sw_vel),"<<pathPos<<","<<velocitySwitchingPoint.pathPos<<","<<velocitySwitchingPoint.pathVel<<std::endl;

	if(accelerationReachedEnd && velocityReachedEnd) {
		return true;
	}
	else if(!accelerationReachedEnd && (velocityReachedEnd || accelerationSwitchingPoint.pathPos <= velocitySwitchingPoint.pathPos)) {
		nextSwitchingPoint = accelerationSwitchingPoint;
		beforeAcceleration = accelerationBeforeAcceleration;
		afterAcceleration = accelerationAfterAcceleration;

		// COMP
		// std::cout<<"CPP next_sw_point (pos_along_path;sw_pos;sw_vel;before_accel;after_accel),"<<pathPos<<","<<nextSwitchingPoint.pathPos<<","<<nextSwitchingPoint.pathVel<<","<<beforeAcceleration<<","<<afterAcceleration<<std::endl;

		return false;
	}
	else {
		nextSwitchingPoint = velocitySwitchingPoint;
		beforeAcceleration = velocityBeforeAcceleration;
		afterAcceleration = velocityAfterAcceleration;

		// COMP
		// std::cout<<"CPP next_sw_point (pos_along_path;sw_pos;sw_vel;before_accel;after_accel),"<<pathPos<<","<<nextSwitchingPoint.pathPos<<","<<nextSwitchingPoint.pathVel<<","<<beforeAcceleration<<","<<afterAcceleration<<std::endl;

		return false;
	}
}

bool Trajectory::getNextAccelerationSwitchingPoint(double pathPos, TrajectoryStep &nextSwitchingPoint, double &beforeAcceleration, double &afterAcceleration) {
	double switchingPathPos = pathPos;
	double switchingPathVel;
	while(true) {
		bool discontinuity;
		switchingPathPos = path.getNextSwitchingPoint(switchingPathPos, discontinuity);

		if(switchingPathPos > path.getLength() - eps) {
			return true;
		}

		if(discontinuity) {
			const double beforePathVel = getAccelerationMaxPathVelocity(switchingPathPos - eps);
			const double afterPathVel = getAccelerationMaxPathVelocity(switchingPathPos + eps);
			switchingPathVel = min(beforePathVel, afterPathVel);
			beforeAcceleration = getMinMaxPathAcceleration(switchingPathPos - eps, switchingPathVel, false);
			afterAcceleration = getMinMaxPathAcceleration(switchingPathPos + eps, switchingPathVel, true);

			if((beforePathVel > afterPathVel
				|| getMinMaxPhaseSlope(switchingPathPos - eps, switchingPathVel, false) > getAccelerationMaxPathVelocityDeriv(switchingPathPos - 2.0*eps))
				&& (beforePathVel < afterPathVel
				|| getMinMaxPhaseSlope(switchingPathPos + eps, switchingPathVel, true) < getAccelerationMaxPathVelocityDeriv(switchingPathPos + 2.0*eps)))
			{
				// COMP
				std::cout<<"CPP acc_sw_discont (in_pos;next_post;next_vel),"<<pathPos<<","<<switchingPathPos<<","<<switchingPathVel<<std::endl;
				break;
			}
		}
		else {
			switchingPathVel = getAccelerationMaxPathVelocity(switchingPathPos);
			beforeAcceleration = 0.0;
			afterAcceleration = 0.0;

			if(getAccelerationMaxPathVelocityDeriv(switchingPathPos - eps) < 0.0 && getAccelerationMaxPathVelocityDeriv(switchingPathPos + eps) > 0.0) {
				// COMP
				// std::cout<<"CPP acc_sw_cont (in_pos;next_post;next_vel),"<<pathPos<<","<<switchingPathPos<<","<<switchingPathVel<<std::endl;
				break;
			}
		}
	}

	nextSwitchingPoint = TrajectoryStep(switchingPathPos, switchingPathVel);
	return false;
}

// Search along path for next switching point. This method first finds a reasonably inaccurate interval in which the switching point occurs, defined by
// the `stepSize` variable. Then, another loop binary searches the interval to find the switching point to within an interval of `accuracy`.
bool Trajectory::getNextVelocitySwitchingPoint(double pathPos, TrajectoryStep &nextSwitchingPoint, double &beforeAcceleration, double &afterAcceleration) {
	const double stepSize = 0.001;
	const double accuracy = 0.000001;

	// Whether the end (yes, the end) of the interval has been found
	bool start = false;
	pathPos -= stepSize;

	// Find the path position just _after_ the next switching point
	do {
		pathPos += stepSize;


		if(getMinMaxPhaseSlope(pathPos, getVelocityMaxPathVelocity(pathPos), false) >= getVelocityMaxPathVelocityDeriv(pathPos)) {
			start = true;
		}
	} while((!start || getMinMaxPhaseSlope(pathPos, getVelocityMaxPathVelocity(pathPos), false) > getVelocityMaxPathVelocityDeriv(pathPos))
		&& pathPos < path.getLength());

	// COMP
	// std::cout<<"CPP end_condition (pathPos;path.getLength()),"<<pathPos<<","<<path.getLength()<<std::endl;

	// No next switching point could be found (return `None`)
	if(pathPos >= path.getLength()) {
		return true; // end of trajectory reached
	}

	// Create a interval around the switching point to speed up finding its more precise location
	double beforePathPos = pathPos - stepSize;
	double afterPathPos = pathPos;

	// Sort-of binary search through interval to find switching point with a given accuracy
	// Presumably reducing accuracy will speed up algo at cost of precision
	while(afterPathPos - beforePathPos > accuracy) {
		// Set path pos to midpoint of search space
		pathPos = (beforePathPos + afterPathPos) / 2.0;

		if(getMinMaxPhaseSlope(pathPos, getVelocityMaxPathVelocity(pathPos), false) > getVelocityMaxPathVelocityDeriv(pathPos)) {
			// Reduce beginning of search space to current position
			beforePathPos = pathPos;
		}
		else {
			// Reduce end of search space to current position
			afterPathPos = pathPos;
		}
	}

	beforeAcceleration = getMinMaxPathAcceleration(beforePathPos, getVelocityMaxPathVelocity(beforePathPos), false);
	afterAcceleration = getMinMaxPathAcceleration(afterPathPos, getVelocityMaxPathVelocity(afterPathPos), true);
	nextSwitchingPoint = TrajectoryStep(afterPathPos, getVelocityMaxPathVelocity(afterPathPos));
	return false;
}

// returns true if end of path is reached
bool Trajectory::integrateForward(list<TrajectoryStep> &trajectory, double acceleration) {

	// Start at the end of the existing trajectory
	double pathPos = trajectory.back().pathPos;
	double pathVel = trajectory.back().pathVel;

	list<pair<double, bool> > switchingPoints = path.getSwitchingPoints();
	list<pair<double, bool> >::iterator nextDiscontinuity = switchingPoints.begin();

	while(true)
	{
		// Search through list of switching points to find the next point after the current position that is discontinuous
		while(nextDiscontinuity != switchingPoints.end() && (nextDiscontinuity->first <= pathPos || !nextDiscontinuity->second)) {
			nextDiscontinuity++;
		}

		// Store previous position
		double oldPathPos = pathPos;
		double oldPathVel = pathVel;

		// Step forward
		pathVel += timeStep * acceleration;
		pathPos += timeStep * 0.5 * (oldPathVel + pathVel);

		// If there is a next switching point and the current position is after it, err do something
		if(nextDiscontinuity != switchingPoints.end() && pathPos > nextDiscontinuity->first) {
			pathVel = oldPathVel + (nextDiscontinuity->first - oldPathPos) * (pathVel - oldPathVel) / (pathPos - oldPathPos);
			pathPos = nextDiscontinuity->first;
		}

		// Reached end of path, break
		if(pathPos > path.getLength()) {
			trajectory.push_back(TrajectoryStep(pathPos, pathVel));
			return true;
		}
		else if(pathVel < 0.0) {
			// Velocity can't be zero; set error flag and bail
			valid = false;
			cout << "Error: Velocity less than zero: " << pathVel << " at position " << pathPos << endl;
			return true;
		}

		// Clamp path velocity to max velocity (and check something else with the phase slope. Not sure what)
		if(pathVel > getVelocityMaxPathVelocity(pathPos)
			&& getMinMaxPhaseSlope(oldPathPos, getVelocityMaxPathVelocity(oldPathPos), false) <= getVelocityMaxPathVelocityDeriv(oldPathPos))
		{
			pathVel = getVelocityMaxPathVelocity(pathPos);
		}

		// Add trajectory step
		trajectory.push_back(TrajectoryStep(pathPos, pathVel));
		// Update acceleration
		acceleration = getMinMaxPathAcceleration(pathPos, pathVel, true);

		// If the velocity has overshot limits, walk back and do a more accurate search to stop overshoot
		if(pathVel > getAccelerationMaxPathVelocity(pathPos) || pathVel > getVelocityMaxPathVelocity(pathPos)) {
			// find more accurate intersection with max-velocity curve using bisection
			TrajectoryStep overshoot = trajectory.back();
			// Remove overshooting trajectory step
			trajectory.pop_back();

			// Create an interval between good place on path and overshoot
			double before = trajectory.back().pathPos;
			double beforePathVel = trajectory.back().pathVel;
			double after = overshoot.pathPos;
			double afterPathVel = overshoot.pathVel;

			// Binary search interval, find place where velocity is at or below limit
			while(after - before > eps) {
				const double midpoint = 0.5 * (before + after);
				double midpointPathVel = 0.5 * (beforePathVel + afterPathVel);

				if(midpointPathVel > getVelocityMaxPathVelocity(midpoint)
					&& getMinMaxPhaseSlope(before, getVelocityMaxPathVelocity(before), false) <= getVelocityMaxPathVelocityDeriv(before))
				{
					midpointPathVel = getVelocityMaxPathVelocity(midpoint);
				}

				// Velocity is too high, reduce search space to lower half
				if(midpointPathVel > getAccelerationMaxPathVelocity(midpoint) || midpointPathVel > getVelocityMaxPathVelocity(midpoint)) {
					after = midpoint;
					afterPathVel = midpointPathVel;
				}
				// Velocity is in second half
				else {
					before = midpoint;
					beforePathVel = midpointPathVel;
				}
			}

			// Re-add last point, but with new, clamped velocity
			trajectory.push_back(TrajectoryStep(before, beforePathVel));

			// Loop end conditions; TODO: Explain
			if(getAccelerationMaxPathVelocity(after) < getVelocityMaxPathVelocity(after)) {
				if(after > nextDiscontinuity->first) {
					return false;
				}
				else if(getMinMaxPhaseSlope(trajectory.back().pathPos, trajectory.back().pathVel, true)
					> getAccelerationMaxPathVelocityDeriv(trajectory.back().pathPos))
				{
					return false;
				}
			}
			else {
				if(getMinMaxPhaseSlope(trajectory.back().pathPos, trajectory.back().pathVel, false)
					> getVelocityMaxPathVelocityDeriv(trajectory.back().pathPos))
				{
					return false;
				}
			}
		}
	}
}

// Integrate backwards along the path, starting from the next switching point and ending at any found intersection.
// If no intersection is found, the algorithm has failed
void Trajectory::integrateBackward(list<TrajectoryStep> &startTrajectory, double pathPos, double pathVel, double acceleration) {
	list<TrajectoryStep>::iterator start2 = startTrajectory.end();
	start2--;
	list<TrajectoryStep>::iterator start1 = start2;
	start1--;
	list<TrajectoryStep> trajectory;
	double slope = 0.0;
	assert(start1->pathPos <= pathPos);
	int blah = startTrajectory.size() - 1;
	double start_sw_pos = pathPos;
	// Move backwards through path in windows of 2. Exit if beginning of path is reached
	while(start1 != startTrajectory.begin() || pathPos >= 0.0)
	{
		// Walk backwards through current segment by self.timestep
		if(start1->pathPos <= pathPos) {
			// Add new trajectory step of current position
			trajectory.push_front(TrajectoryStep(pathPos, pathVel));

			// Step backwards along the path by self.timestep, update pos/vel/acc at new position's
			pathVel -= timeStep * acceleration;
			// Update position with average of previous and current velocity
			pathPos -= timeStep * 0.5 * (pathVel + trajectory.front().pathVel);
			// Get _minimum_ (max = false) acceleration at new point
			acceleration = getMinMaxPathAcceleration(pathPos, pathVel, false);
			// Update slope at new position
			slope = (trajectory.front().pathVel - pathVel) / (trajectory.front().pathPos - pathPos);

			// COMP
			std::cout<<"CPP back_step (pathPos;pathVel;acceleration;slope),"<<pathPos<<","<<pathVel<<","<<acceleration<<","<<slope<<std::endl;

			// If velocity is below zero, bail
			if(pathVel < 0.0) {
				valid = false;
				cout << "Error while integrating backward: Negative path velocity" << endl;
				endTrajectory = trajectory;
				return;
			}
		}
		// If position now lies before current segment, move to the previous segment to step through that
		else {
			start1--;
			start2--;

			blah--;
		}

		// For every step through the segment, check for intersection between current start trajectory and backward trajectory segments
		// Find slope of current segment
		const double startSlope = (start2->pathVel - start1->pathVel) / (start2->pathPos - start1->pathPos);
		// Find point along path where backwards integration intersects forward integration
		const double intersectionPathPos = (start1->pathVel - pathVel + slope * pathPos - startSlope * start1->pathPos) / (slope - startSlope);

		// COMP
		std::cout<<"CPP intersection_values (i_pos;start_slope),"<<intersectionPathPos<<","<<startSlope<<std::endl;

		// If the current backwards-iterating segment intersects the original path, we're done
		if(max(start1->pathPos, pathPos) - eps <= intersectionPathPos && intersectionPathPos <= eps + min(start2->pathPos, trajectory.front().pathPos)) {
			const double intersectionPathVel = start1->pathVel + startSlope * (intersectionPathPos - start1->pathPos);
			// Remove the part of the path after the intersection
			startTrajectory.erase(start2, startTrajectory.end());
			// Add the intersection point
			startTrajectory.push_back(TrajectoryStep(intersectionPathPos, intersectionPathVel));
			// Append this newly generated path to the current trajectory
			startTrajectory.splice(startTrajectory.end(), trajectory);

			// COMP
			std::cout<<"CPP back_splice_idx (start_sw_pos;splice_idx),"<<start_sw_pos<<","<<blah<<std::endl;

			return;
		}
	}

	// Original path was not intersected. In this case the algorithm failed so should bail
	valid = false;
	cout << "Error while integrating backward: Did not hit start trajectory" << endl;
	endTrajectory = trajectory;
}

double Trajectory::getMinMaxPathAcceleration(double pathPos, double pathVel, bool max) {
	Vector3d configDeriv = path.getTangent(pathPos);
	Vector3d configDeriv2 = path.getCurvature(pathPos);

	// std::cout<<"CPP deriv1,"<<pathPos<<","<<configDeriv[0]<<","<<configDeriv[1]<<","<<configDeriv[2]<<std::endl;
	// std::cout<<"CPP deriv2,"<<pathPos<<","<<configDeriv2[0]<<","<<configDeriv2[1]<<","<<configDeriv2[2]<<std::endl;

	double factor = max ? 1.0 : -1.0;
	double maxPathAcceleration = numeric_limits<double>::max();
	for(unsigned int i = 0; i < n; i++) {
		if(configDeriv[i] != 0.0) {
			maxPathAcceleration = min(maxPathAcceleration,
				maxAcceleration[i]/abs(configDeriv[i]) - factor * configDeriv2[i] * pathVel*pathVel / configDeriv[i]);
		}
	}

	// COMP
	std::cout<<"CPP acc_at (pathPos;pathVel;factor*maxPathAcceleration),"<<pathPos<<","<<pathVel<<","<<(factor*maxPathAcceleration)<<std::endl;
	return factor * maxPathAcceleration;
}

double Trajectory::getMinMaxPhaseSlope(double pathPos, double pathVel, bool max) {
	return getMinMaxPathAcceleration(pathPos, pathVel, max) / pathVel;
}

double Trajectory::getAccelerationMaxPathVelocity(double pathPos) const {
	double maxPathVelocity = numeric_limits<double>::infinity();
	const Vector3d configDeriv = path.getTangent(pathPos);
	const Vector3d configDeriv2 = path.getCurvature(pathPos);
	for(unsigned int i = 0; i < n; i++) {
		if(configDeriv[i] != 0.0) {
			for(unsigned int j = i + 1; j < n; j++) {
				if(configDeriv[j] != 0.0) {
					double A_ij = configDeriv2[i] / configDeriv[i] - configDeriv2[j] / configDeriv[j];
					if(A_ij != 0.0) {
						maxPathVelocity = min(maxPathVelocity,
							sqrt((maxAcceleration[i] / abs(configDeriv[i]) + maxAcceleration[j] / abs(configDeriv[j]))
							/ abs(A_ij)));
					}
				}
			}
		}
		else if(configDeriv2[i] != 0.0) {
			maxPathVelocity = min(maxPathVelocity, sqrt(maxAcceleration[i] / abs(configDeriv2[i])));
		}
	}

	// COMP
	std::cout<<"CPP max_vel_from_acc (pos;vel),"<<pathPos<<","<<maxPathVelocity<<std::endl;

	return maxPathVelocity;
}


double Trajectory::getVelocityMaxPathVelocity(double pathPos) const {
	const Vector3d tangent = path.getTangent(pathPos);
	double maxPathVelocity = numeric_limits<double>::max();
	for(unsigned int i = 0; i < n; i++) {
		maxPathVelocity = min(maxPathVelocity, maxVelocity[i] / abs(tangent[i]));
	}

	// COMP
	std::cout<<"CPP max_vel_from_vel (pos;vel),"<<pathPos<<","<<maxPathVelocity<<std::endl;

	return maxPathVelocity;
}

double Trajectory::getAccelerationMaxPathVelocityDeriv(double pathPos) {
	return (getAccelerationMaxPathVelocity(pathPos + eps) - getAccelerationMaxPathVelocity(pathPos - eps)) / (2.0 * eps);
}

double Trajectory::getVelocityMaxPathVelocityDeriv(double pathPos) {
	const Vector3d tangent = path.getTangent(pathPos);
	double maxPathVelocity = numeric_limits<double>::max();
	unsigned int activeConstraint = 0;
	for(unsigned int i = 0; i < n; i++) {
		const double thisMaxPathVelocity = maxVelocity[i] / abs(tangent[i]);
		if(thisMaxPathVelocity < maxPathVelocity) {
			maxPathVelocity = thisMaxPathVelocity;
			activeConstraint = i;
		}
	}
	const double result = - (maxVelocity[activeConstraint] * path.getCurvature(pathPos)[activeConstraint])
		/ (tangent[activeConstraint] * abs(tangent[activeConstraint]));

	// COMP
	std::cout<<"CPP max_vel_vel_deriv (pos;vel),"<<pathPos<<","<<result<<std::endl;

	return result;
}

bool Trajectory::isValid() const {
	return valid;
}

double Trajectory::getDuration() const {
	return trajectory.back().time;
}

list<Trajectory::TrajectoryStep>::const_iterator Trajectory::getTrajectorySegment(double time) const {
	if(time >= trajectory.back().time) {
		list<TrajectoryStep>::const_iterator last = trajectory.end();
		last--;
		return last;
	}
	else {
		if(time < cachedTime) {
			cachedTrajectorySegment = trajectory.begin();
		}
		while(time >= cachedTrajectorySegment->time) {
			cachedTrajectorySegment++;
		}
		cachedTime = time;
		return cachedTrajectorySegment;
	}
}

Vector3d Trajectory::getPosition(double time) const {
	list<TrajectoryStep>::const_iterator it = getTrajectorySegment(time);
	list<TrajectoryStep>::const_iterator previous = it;
	previous--;

	// COMP
	std::cout<<"CPP get_pos (time;prev_pos;prev_vel;curr_pos;curr_vel),"<<time<<","<<previous->pathPos<<","<<previous->pathVel<<","<<it->pathPos<<","<<it->pathVel <<std::endl;
	double timeStep = it->time - previous->time;
	const double acceleration = 2.0
		* (it->pathPos - previous->pathPos - timeStep * previous->pathVel)
		/ (timeStep * timeStep);

	timeStep = time - previous->time;
	const double pathPos = previous->pathPos
		+ timeStep * previous->pathVel
		+ 0.5 * timeStep * timeStep * acceleration;

	// std::cout << "CPP time " << time << ", pos " << pathPos << std::endl;
	// std::cout << "    Previous (pos, vel, t) " << previous->pathPos << "," << previous->pathVel << "," << previous->time << ", current (pos, vel, t) " << it->pathPos << "," << it->pathVel << "," << it->time << std::endl;
	// std::cout << "    timestep  " << timeStep << ", acceleration " << acceleration << std::endl;

	return path.getConfig(pathPos);
}

Vector3d Trajectory::getVelocity(double time) const {
	list<TrajectoryStep>::const_iterator it = getTrajectorySegment(time);
	list<TrajectoryStep>::const_iterator previous = it;
	previous--;

	double timeStep = it->time - previous->time;
	const double acceleration = 2.0
		* (it->pathPos - previous->pathPos - timeStep * previous->pathVel)
		/ (timeStep * timeStep);

	timeStep = time - previous->time;
	const double pathPos = previous->pathPos
		+ timeStep * previous->pathVel
		+ 0.5 * timeStep * timeStep * acceleration;
	const double pathVel = previous->pathVel + timeStep * acceleration;

	return path.getTangent(pathPos) * pathVel;
}
