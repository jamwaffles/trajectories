#include <vector>
#include <list>
#include <Eigen/Core>

#include "Path.hpp"
#include "Trajectory.hpp"

// Take a C-style pointer to a list of floats and make a Path with it
extern "C" void* create_path(float *waypoints, float step) {
	int len = sizeof(waypoints) / sizeof(waypoints[0]);

	std::vector<float> nums(waypoints, waypoints + len);

	std::list<Eigen::Vector3f> wps;

	for (auto i = nums.begin(); i != nums.end();) {
		Eigen::Vector3f wp;

		wp << *i;
		i++;
		wp << *i;
		i++;
		wp << *i;
		i++;

		wps.push_back(wp);
	}

	return new Path(wps, step);
}