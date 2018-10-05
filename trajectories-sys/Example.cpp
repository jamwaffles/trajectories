/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
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

#include <iostream>
#include <iomanip>
#include <Eigen/Core>
#include "Trajectory.hpp"
#include "Path.hpp"

using namespace std;
using namespace Eigen;

int main() {
	list<Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > waypoints;
	Vector3d waypoint;
	waypoint << 0.0, 0.0, 0.0;
	waypoints.push_back(waypoint);
	waypoint << 0.0, 0.2, 1.0;
	waypoints.push_back(waypoint);
	waypoint << 0.0, 3.0, 0.5;
	waypoints.push_back(waypoint);
	waypoint << 1.1, 2.0, 0.0;
	waypoints.push_back(waypoint);
	waypoint << 1.0, 0.0, 0.0;
	waypoints.push_back(waypoint);
	waypoint << 0.0, 1.0, 0.0;
	waypoints.push_back(waypoint);
	waypoint << 0.0, 0.0, 1.0;
	waypoints.push_back(waypoint);

	Vector3d maxAcceleration(3);
	maxAcceleration << 1.0, 1.0, 1.0;
	Vector3d maxVelocity(3);
	maxVelocity << 1.0, 1.0, 1.0;

	Trajectory trajectory(Path(waypoints, 0.001), maxVelocity, maxAcceleration);
	trajectory.outputPhasePlaneTrajectory();
	if(trajectory.isValid()) {
		double duration = trajectory.getDuration();
		// cout << "Trajectory duration: " << duration << " s" << endl << endl;
		cout << "time,position_x,position_y,position_z,velocity_x,velocity_y,velocity_z" << endl;
		for(double t = 0.0; t < duration; t += 0.1) {
			Vector3d pos = trajectory.getPosition(t);
			Vector3d vel = trajectory.getVelocity(t);

			std::cout << fixed << std::setprecision(17)
			<< t << ","
			<< pos[0] << ","
			<< pos[1] << ","
			<< pos[2] << ","
			<< vel[0] << ","
			<< vel[1] << ","
			<< vel[2]
			<< std::endl;
		}

		Vector3d pos = trajectory.getPosition(duration);
		Vector3d vel = trajectory.getVelocity(duration);

		std::cout << fixed << std::setprecision(17)
		<< duration << ","
		<< pos[0] << ","
		<< pos[1] << ","
		<< pos[2] << ","
		<< vel[0] << ","
		<< vel[1] << ","
		<< vel[2]
		<< std::endl;
	}
	else {
		cout << "Trajectory generation failed." << endl;
	}

	return 0;
}


