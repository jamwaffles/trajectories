#include <vector>
#include <list>
#include <iostream>
#include <Eigen/Core>

#include "Path.hpp"
#include "Trajectory.hpp"

// Take a C-style pointer to a list of floats and make a Path with it
extern "C" void* path_create(double *waypoints, int len, double step) {
	// int n = sizeof(waypoints) / sizeof(waypoints[0]);

	// std::vector<double> nums(waypoints, waypoints + n);
	std::vector<double> nums;
	nums.assign(waypoints, waypoints + len);

	std::list<Eigen::Vector3d> wps;

	// std::cout << "NUM " << nums.size() << std::endl;

	for (auto i = nums.begin(); i != nums.end();) {
		Eigen::Vector3d wp;

		double x = 0.0;
		double y = 0.0;
		double z = 0.0;

		x = *i;
		i++;

		y = *i;
		i++;

		z = *i;
		i++;

		// std::cout << "X " << x << std::endl;
		// std::cout << "Y " << y << std::endl;
		// std::cout << "Z " << z << std::endl;

		wp << x, y, z;

		wps.push_back(wp);
	}

	// std::cout << "WAYPOINTS " << wps.size() << std::endl;

	return new Path(wps, step);
}
