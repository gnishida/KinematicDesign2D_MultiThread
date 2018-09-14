﻿#include "LinkageSynthesisRRRP.h"
#include "KinematicUtils.h"
#include "Kinematics.h"
#include "PinJoint.h"
#include "SliderHinge.h"
#include "BoundingBox.h"
#include "LeastSquareSolver.h"
#include <opencv2/opencv.hpp>
#include <boost/thread.hpp>

namespace kinematics {
	
	LinkageSynthesisRRRP::LinkageSynthesisRRRP(const std::vector<Object25D>& fixed_bodies, const std::pair<double, double>& sigmas, bool avoid_branch_defect, double min_transmission_angle, double min_link_length, const std::vector<double>& weights) {
		this->fixed_bodies = fixed_bodies;
		this->sigmas = sigmas;
		this->avoid_branch_defect = avoid_branch_defect;
		this->min_transmission_angle = min_transmission_angle;
		this->min_link_length = min_link_length;
		this->weights = weights;
	}

	/**
	* Calculate solutions of RRRP linkage given three poses.
	*
	* @param poses			three poses
	* @param solutions1	the output solutions for the driving crank, each of which contains a pair of the center point and the circle point
	* @param solutions2	the output solutions for the follower, each of which contains a pair of the fixed point and the slider point
	*/
	void LinkageSynthesisRRRP::calculateSolution(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& linkage_region_pts, const std::vector<glm::dvec2>& linkage_avoidance_pts, int num_samples, const Object25D& moving_body, std::vector<Solution>& solutions) {
		solutions.clear();

		srand(0);

		// calculate the bounding boxe of the valid regions
		BBox bbox = boundingBox(linkage_region_pts);
			
		// run multi-thread for sampling
		const int NUM_THREADS = 8;
		std::vector<boost::thread> threads(NUM_THREADS);
		std::vector<std::vector<Solution>> sub_solutions(NUM_THREADS);
		for (int i = 0; i < threads.size(); i++) {
			threads[i] = boost::thread(&LinkageSynthesisRRRP::calculateSolutionThread, this, i, boost::ref(poses), boost::ref(linkage_region_pts), boost::ref(bbox), boost::ref(linkage_avoidance_pts), num_samples / NUM_THREADS, boost::ref(moving_body), boost::ref(sub_solutions[i]));
		}
		for (int i = 0; i < threads.size(); i++) {
			threads[i].join();
		}

		// merge the obtained solutions
		for (int i = 0; i < sub_solutions.size(); i++) {
			solutions.insert(solutions.end(), sub_solutions[i].begin(), sub_solutions[i].end());
		}
	}

	void LinkageSynthesisRRRP::calculateSolutionThread(int thread_id, const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& linkage_region_pts, const BBox& bbox, const std::vector<glm::dvec2>& linkage_avoidance_pts, int num_samples, const Object25D& moving_body, std::vector<Solution>& solutions) {
		std::default_random_engine generator(thread_id);

		for (int iter = 0; iter < num_samples && solutions.size() < 50; iter++) {
			// perturbe the poses a little
			double position_error = 0.0;
			double orientation_error = 0.0;
			std::vector<glm::dmat3x3> perturbed_poses = perturbPoses(poses, sigmas, generator, position_error, orientation_error);

			// sample joints within the linkage region
			std::vector<glm::dvec2> points(5);
			for (int i = 0; i < points.size(); i++) {
				while (true) {
					points[i] = glm::dvec2(genRand(generator, bbox.minPt.x, bbox.maxPt.x), genRand(generator, bbox.minPt.y, bbox.maxPt.y));
					if (withinPolygon(linkage_region_pts, points[i])) break;
				}
			}

			// HACK: this indicates that the positions of the joint are random
			//       for the particle filter, the initial joints are used to optimize, so we want to differentiate these two cases.
			//points[1] = points[3];

			if (!optimizeCandidate(perturbed_poses, points)) continue;

			// check hard constraints
			if (!checkHardConstraints(points, perturbed_poses, linkage_avoidance_pts, moving_body)) continue;

			solutions.push_back(Solution(1, points, position_error, orientation_error, perturbed_poses));
		}
	}

	/**
	* Optimize the linkage parameter based on the rigidity constraints.
	* If it fails to opotimize, return false.
	*/
	bool LinkageSynthesisRRRP::optimizeCandidate(const std::vector<glm::dmat3x3>& poses, std::vector<glm::dvec2>& points) {
		if (!optimizeLink(poses, points[0], points[2])) return false;
		if (!optimizeSlider(poses, points[1], points[3])) return false;

		return true;
	}

	bool LinkageSynthesisRRRP::optimizeLink(const std::vector<glm::dmat3x3>& poses, glm::dvec2& A0, glm::dvec2& A1) {
		// setup the initial parameters for optimization
		column_vector starting_point(4);
		starting_point(0, 0) = A0.x;
		starting_point(1, 0) = A0.y;
		starting_point(2, 0) = A1.x;
		starting_point(3, 0) = A1.y;

		try {
			find_min(dlib::bfgs_search_strategy(), dlib::objective_delta_stop_strategy(1e-7), SolverForLink(poses), SolverDerivForLink(poses), starting_point, -1);

			A0.x = starting_point(0, 0);
			A0.y = starting_point(1, 0);
			A1.x = starting_point(2, 0);
			A1.y = starting_point(3, 0);
		}
		catch (std::exception& e) {
			return false;
		}

		return true;
	}

	bool LinkageSynthesisRRRP::optimizeSlider(const std::vector<glm::dmat3x3>& poses, glm::dvec2& A0, glm::dvec2& A1) {
		// setup the initial parameters for optimization
		column_vector starting_point(4);
		starting_point(0, 0) = A0.x;
		starting_point(1, 0) = A0.y;
		starting_point(2, 0) = A1.x;
		starting_point(3, 0) = A1.y;

		try {
			find_min_using_approximate_derivatives(dlib::bfgs_search_strategy(), dlib::objective_delta_stop_strategy(1e-7), SolverForSlider(poses), starting_point, -1);

			A1.x = starting_point(2, 0);
			A1.y = starting_point(3, 0);
		}
		catch (std::exception& e) {
			return false;
		}

		// calculate the local coordinate of A1
		glm::dvec2 a = glm::dvec2(glm::inverse(poses[0]) * glm::dvec3(A1, 1));

		glm::dvec2 v1 = glm::dvec2(poses[1] * glm::dvec3(a, 1)) - A1;
		double l1 = glm::length(v1);
		v1 /= l1;

		for (int i = 2; i < poses.size(); i++) {
			glm::dvec2 A(poses[i] * glm::dvec3(a, 1));
			glm::dvec2 v = A - A1;
			double l = glm::length(v);

			// check the collinearity
			if (abs(crossProduct(v1, v / l)) > 0.01) return false;

			// check the order
			l = glm::dot(v1, v);
			if (l <= 0) return false;
			if (l <= l1) return false;
		}

		A0 = A1 - v1;

		return true;
	}

	Solution LinkageSynthesisRRRP::findBestSolution(const std::vector<glm::dmat3x3>& poses, std::vector<Solution>& solutions, const cv::Mat& dist_map, const BBox& dist_map_bbox, const std::vector<glm::dvec2>& linkage_avoidance_pts, const Object25D& moving_body, int num_particles, int num_iterations, bool record_file) {
		// select the best solution based on the objective function
		if (solutions.size() > 0) {
			particleFilter(poses, solutions, dist_map, dist_map_bbox, linkage_avoidance_pts, moving_body, num_particles, num_iterations, record_file);
			return solutions[0];
		}
		else {
			return Solution(1, { { 0, 0 }, { 0, 2 }, { 2, 0 }, { 2, 2 }, { 4, 2 } }, 0, 0, poses);
		}
	}

	double LinkageSynthesisRRRP::calculateCost(Solution& solution, const Object25D& moving_body, const cv::Mat& dist_map, const BBox& dist_map_bbox) {
		double dist = 0;
		for (int i = 0; i < solution.points.size(); i++) {
			dist += dist_map.at<double>(solution.points[i].y - dist_map_bbox.minPt.y, solution.points[i].x - dist_map_bbox.minPt.x);
		}
		double tortuosity = tortuosityOfTrajectory(solution.poses, solution.points, moving_body);
		std::vector<glm::dvec2> connected_pts;
		Kinematics kin = constructKinematics(solution.poses, solution.points, moving_body, true, fixed_bodies, connected_pts);
		//double size = glm::length(solution.points[0] - solution.points[2]) + glm::length(solution.points[1] - solution.points[3]) + glm::length(solution.points[0] - connected_pts[0]) + glm::length(solution.points[1] - connected_pts[1]) + glm::length(solution.points[4] - connected_pts[2]) + glm::length(solution.points[2] - connected_pts[3]) + glm::length(solution.points[3] - connected_pts[4]);
		double size = glm::length(solution.points[0] - solution.points[2]) + glm::length(solution.points[1] - solution.points[4]) + glm::length(solution.points[0] - connected_pts[0]) + glm::length(solution.points[1] - connected_pts[1]) + glm::length(solution.points[4] - connected_pts[2]) + glm::length(solution.points[2] - connected_pts[3]) + glm::length(solution.points[3] - connected_pts[4]);

		return solution.position_error * weights[0] + solution.orientation_error * weights[0] * 10 + dist * weights[1] + (tortuosity - 1) * weights[2] + size * weights[3];
	}

	/**
	* Construct a linkage.
	*/
	Kinematics LinkageSynthesisRRRP::constructKinematics(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& points, const Object25D& moving_body, bool connect_joints, const std::vector<Object25D>& fixed_bodies, std::vector<glm::dvec2>& connected_pts) {
		kinematics::Kinematics kin;
		kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(0, true, points[0])));
		kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(1, true, points[1])));
		kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(2, false, points[2])));
		kin.diagram.addJoint(boost::shared_ptr<kinematics::SliderHinge>(new kinematics::SliderHinge(3, false, points[3])));
		kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(4, true, points[4])));
		kin.diagram.addLink(true, kin.diagram.joints[0], kin.diagram.joints[2], true);
		kin.diagram.addLink(false, { kin.diagram.joints[1], kin.diagram.joints[3], kin.diagram.joints[4] }, true);
		kin.diagram.addLink(false, kin.diagram.joints[2], kin.diagram.joints[3], false);

		std::vector<Object25D> copied_fixed_bodies = fixed_bodies;

		// update the geometry
		updateBodies(kin, moving_body);

		if (connect_joints) {
			kin.diagram.connectJointsToBodies(copied_fixed_bodies, connected_pts);
		}

		// add the fixed rigid bodies
		for (int i = 0; i < copied_fixed_bodies.size(); i++) {
			kin.diagram.addBody(kin.diagram.joints[0], kin.diagram.joints[1], copied_fixed_bodies[i]);
		}

		// calculte the range of motion
		std::pair<double, double> angle_range = checkRange(poses, points);
		kin.min_angle = angle_range.first;
		kin.max_angle = angle_range.second;

		return kin;
	}

	/**
	* update bodies.
	*/
	void LinkageSynthesisRRRP::updateBodies(Kinematics& kin, const Object25D& moving_body) {
		kin.diagram.bodies.clear();
		kin.diagram.addBody(kin.diagram.joints[2], kin.diagram.joints[3], moving_body);
	}

	bool LinkageSynthesisRRRP::checkHardConstraints(std::vector<glm::dvec2>& points, const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& linkage_avoidance_pts, const Object25D& moving_body) {
		glm::dvec2 slider_dir = points[3] - points[1];

		// check hard constraints
		if (glm::length(points[0] - points[1]) < min_link_length) return false;
		if (glm::length(points[2] - points[3]) < min_link_length) return false;

		if (avoid_branch_defect && checkBranchDefect(poses, points)) return false;
		if (checkCircuitDefect(poses, points)) return false;

		// collision check
		// moving_body[0] means the main body without the joint connectors
		glm::dvec2 slider_end_pos1, slider_end_pos2;
		if (checkCollision(poses, points, fixed_bodies, moving_body[0], linkage_avoidance_pts, slider_end_pos1, slider_end_pos2)) return false;

		// locate the two endpoints of the bar
		points[1] = slider_end_pos1 - slider_dir * 2.0;
		points[4] = slider_end_pos2 + slider_dir * 2.0;

		return true;
	}

	/**
	* Return the RRRP linkage type.
	*
	* 0 -- rotatable crank
	* 1 -- 0-rocker
	* 2 -- pi-rocker
	* 3 -- rocker
	*/
	int LinkageSynthesisRRRP::getType(const std::vector<glm::dvec2>& points) {
		// obtain the vectors, u (x axis) and v (y axis)
		glm::dvec2 u = points[3] - points[1];
		u /= glm::length(u);

		glm::dvec2 v(-u.y, u.x);
		if (glm::dot(points[0] - points[1], v) < 0) {
			u = -u;
			v = -v;
		}

		// calculate each length
		double e = glm::dot(points[0] - points[1], v);
		double r = glm::length(points[2] - points[0]);
		double l = glm::length(points[3] - points[2]);

		// calculate S1 and S2
		double S1 = l - r + e;
		double S2 = l - r - e;

		// judge the type of the RRRP linkage
		if (S1 >= 0 && S2 >= 0) return 0;
		else if (S1 >= 0 && S2 < 0) {
			// HACK to differentiate 0-rocker from pi-rocker
			if (v.y >= 0) return 1;
			else return 2;
		}
		//else if (S1 < 0 && S2 >= 0) return 2;
		else return 3;
	}

	std::pair<double, double> LinkageSynthesisRRRP::checkRange(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& points) {
		if (poses.size() <= 2) return{ 0, 0 };

		glm::dvec2 dir1 = points[2] - points[0];
		glm::dvec2 dir2 = glm::dvec2(poses[1] * glm::inverse(poses[0]) * glm::dvec3(points[2], 1)) - points[0];
		glm::dvec2 dir3 = glm::dvec2(poses.back() * glm::inverse(poses[0]) * glm::dvec3(points[2], 1)) - points[0];

		double angle1 = std::atan2(dir1.y, dir1.x);
		double angle2 = std::atan2(dir2.y, dir2.x);
		double angle3 = std::atan2(dir3.y, dir3.x);

		// try clock-wise order
		double a1 = angle1;
		double a2 = angle2;
		double a3 = angle3;
		if (a2 > a1) {
			a2 -= M_PI * 2;
		}
		while (a3 > a2) {
			a3 -= M_PI * 2;
		}
		if (a1 - a3 < M_PI * 2) {
			return{ std::min(a1, a3), std::max(a1, a3) };
		}

		// try counter-clock-wise order
		if (angle2 < angle1) {
			angle2 += M_PI * 2;
		}
		if (angle3 < angle2) {
			angle3 += M_PI * 2;
		}
		if (angle3 - angle1 < M_PI * 2) {
			return{ std::min(angle1, angle3), std::max(angle1, angle3) };
		}

		return{ 0, 0 };
	}

	bool LinkageSynthesisRRRP::checkOrderDefect(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& points) {
		return false;
	}

	bool LinkageSynthesisRRRP::checkBranchDefect(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& points) {
		int type = getType(points);

		// rotatable crank always does not have a branch defect
		if (type == 0) return false;

		// obtain the vectors, u (x axis) and v (y axis)
		glm::dvec2 u = points[3] - points[1];
		u /= glm::length(u);

		glm::dvec2 v(-u.y, u.x);
		if (glm::dot(points[0] - points[1], v) < 0) {
			u = -u;
			v = -v;
		}

		int orig_sign = 1;

		// calculate the local coordinates of the circle points
		glm::dvec2 q2 = glm::dvec2(glm::inverse(poses[0]) * glm::dvec3(points[2], 1));
		glm::dvec2 q3 = glm::dvec2(glm::inverse(poses[0]) * glm::dvec3(points[3], 1));

		for (int i = 0; i < poses.size(); i++) {
			// calculate the coordinates of the circle point of the driving/driven cranks in the world coordinate system
			glm::dvec2 P2 = glm::dvec2(poses[i] * glm::dvec3(q2, 1));
			glm::dvec2 P3 = glm::dvec2(poses[i] * glm::dvec3(q3, 1));

			// calculate the sign of the dot product of L and u
			if (i == 0) {
				orig_sign = glm::dot(P3 - P2, u) >= 0 ? 1 : -1;
			}
			else {
				int sign = glm::dot(P3 - P2, u) >= 0 ? 1 : -1;
				if (sign != orig_sign) return true;
			}
		}

		return false;
	}

	bool LinkageSynthesisRRRP::checkCircuitDefect(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& points) {
		int type = getType(points);

		// 0-rocker and pi-rocker always do not have a branch defect
		if (type == 1 || type == 2) return false;

		// obtain the vectors, u (x axis) and v (y axis)
		glm::dvec2 u = points[3] - points[1];
		u /= glm::length(u);

		glm::dvec2 v(-u.y, u.x);
		if (glm::dot(points[0] - points[1], v) < 0) {
			u = -u;
			v = -v;
		}

		int orig_sign = 1;

		// calculate the local coordinates of the circle points
		glm::dvec2 q2 = glm::dvec2(glm::inverse(poses[0]) * glm::dvec3(points[2], 1));
		glm::dvec2 q3 = glm::dvec2(glm::inverse(poses[0]) * glm::dvec3(points[3], 1));

		for (int i = 0; i < poses.size(); i++) {
			// calculate the coordinates of the circle point of the driving/driven cranks in the world coordinate system
			glm::dvec2 P2 = glm::dvec2(poses[i] * glm::dvec3(q2, 1));
			glm::dvec2 P3 = glm::dvec2(poses[i] * glm::dvec3(q3, 1));

			// calculate the sign of the dot product of L and u
			if (i == 0) {
				if (type == 0) {
					orig_sign = glm::dot(P3 - P2, u) >= 0 ? 1 : -1;
				}
				else {
					orig_sign = glm::dot(P2 - points[0], u) >= 0 ? 1 : -1;
				}
			}
			else {
				int sign;
				if (type == 0) {
					sign = glm::dot(P3 - P2, u) >= 0 ? 1 : -1;
				}
				else {
					sign = glm::dot(P2 - points[0], u) >= 0 ? 1 : -1;
				}
				if (sign != orig_sign) return true;
			}
		}

		return false;
	}

	/**
	* Check collision. If there are any collisions, return true.
	* Only the main body is checked for collision.
	*
	* @param poses					N poses
	* @param points				joint coordinates
	* @param fixed_bodies			list of fixed bodies
	* @param moving_body			moving body
	* @param linkage_avoidance_pts	region to avoid for the linkage
	* @return						true if collision occurs
	*/
	bool LinkageSynthesisRRRP::checkCollision(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& points, const std::vector<Object25D>& fixed_bodies, const Object25D& moving_body, const std::vector<glm::dvec2>& linkage_avoidance_pts, glm::dvec2& slider_end_pos1, glm::dvec2& slider_end_pos2) {
		std::vector<glm::dvec2> connector_pts;
		kinematics::Kinematics kinematics = constructKinematics(poses, points, moving_body, false, fixed_bodies, connector_pts);
		kinematics.diagram.initialize();

		// set the initial point of slider and direction
		glm::dvec2 orig_slider_pos = points[3];
		glm::dvec2 slider_dir = glm::normalize(points[3] - points[1]);
		slider_end_pos1 = points[3];
		slider_end_pos2 = points[3];
		double slider_min_dist = 0;
		double slider_max_dist = 0;

		// calculate the rotational angle of the driving crank for 1st, 2nd, and last poses
		// i.e., angles[0] = first pose, angles[1] = second pose, angles[2] = last pose
		std::vector<double> angles(3);
		glm::dvec2 w(glm::inverse(poses[0]) * glm::dvec3(points[2], 1));
		for (int i = 0; i < 2; i++) {
			glm::dvec2 W = glm::dvec2(poses[i] * glm::dvec3(w, 1));
			angles[i] = atan2(W.y - points[0].y, W.x - points[0].x);
		}
		{
			glm::dvec2 W = glm::dvec2(poses.back() * glm::dvec3(w, 1));
			angles[2] = atan2(W.y - points[0].y, W.x - points[0].x);
		}

		// order the angles based on their signs
		int type = 0;
		if (angles[0] < 0 && angles[1] < 0 && angles[2] >= 0 && angles[0] >= angles[1]) {
			type = 1;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] < 0 && angles[1] >= 0 && angles[2] < 0 && angles[0] < angles[2]) {
			type = 2;
			angles[1] -= M_PI * 2;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] < 0 && angles[1] >= 0 && angles[2] < 0 && angles[0] >= angles[2]) {
			type = 3;
			angles[2] += M_PI * 2;
		}
		else if (angles[0] < 0 && angles[1] >= 0 && angles[2] >= 0 && (poses.size() >= 3 && angles[1] >= angles[2] || poses.size() == 2 && angles[1] - angles[0] > M_PI)) {
			type = 4;
			angles[1] -= M_PI * 2;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] < 0 && angles[2] < 0 && (poses.size() >= 3 && angles[1] < angles[2] || poses.size() == 2 && angles[0] - angles[1] > M_PI)) {
			type = 5;
			angles[1] += M_PI * 2;
			angles[2] += M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] < 0 && angles[2] >= 0 && angles[0] < angles[2]) {
			type = 6;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] < 0 && angles[2] >= 0 && angles[0] >= angles[2]) {
			type = 7;
			angles[1] += M_PI * 2;
			angles[2] += M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] >= 0 && angles[2] < 0 && angles[0] < angles[1]) {
			type = 8;
			angles[2] += M_PI * 2;
		}

		if (angles[2] < angles[0]) {
			kinematics.invertSpeed();
		}

		// initialize the visited flag
		std::vector<bool> visited(angles.size(), false);
		visited[0] = true;
		int unvisited = 2;

		// run forward until collision is deteted or all the poses are reached
		while (true) {
			try {
				kinematics.stepForward(true, false);
				double dist = glm::dot(kinematics.diagram.joints[3]->pos - orig_slider_pos, slider_dir);
				if (dist > slider_max_dist) {
					slider_max_dist = dist;
					slider_end_pos2 = kinematics.diagram.joints[3]->pos;
				}
				else if (dist < slider_min_dist) {
					slider_min_dist = dist;
					slider_end_pos1 = kinematics.diagram.joints[3]->pos;
				}
			}
			catch (char* ex) {
				// if only some of the poses are reached before collision, the collision is detected.
				kinematics.clear();
				return true;
			}

			// check if the joints are within the linkage avoidance region
			for (int i = 0; i < kinematics.diagram.joints.size(); i++) {
				if (withinPolygon(linkage_avoidance_pts, kinematics.diagram.joints[i]->pos)) return true;
			}

			// check the transmission angle
			if (avoid_branch_defect) {
				glm::dvec2 a = kinematics.diagram.joints[2]->pos - kinematics.diagram.joints[3]->pos;
				glm::dvec2 b = kinematics.diagram.joints[3]->pos - kinematics.diagram.joints[1]->pos;
				double t_angle = std::acos(glm::dot(a, b) / glm::length(a) / glm::length(b));
				if (std::abs(t_angle) < min_transmission_angle) return true;
			}

			// calculate the angle of the driving crank
			double angle = atan2(kinematics.diagram.joints[2]->pos.y - points[0].y, kinematics.diagram.joints[2]->pos.x - points[0].x);

			// convert the sign of the angle
			if (type == 1 && angle > 0) {
				angle -= M_PI * 2;
			}
			else if (type == 2 && angle > angles[0]) {
				angle -= M_PI * 2;
			}
			else if (type == 3 && angle < angles[0]) {
				angle += M_PI * 2;
			}
			else if (type == 4 && angle > 0) {
				angle -= M_PI * 2;
			}
			else if (type == 5 && angle < 0) {
				angle += M_PI * 2;
			}
			else if (type == 6 && angle > angles[0]) {
				angle -= M_PI * 2;
			}
			else if (type == 7 && angle < angles[0]) {
				angle += M_PI * 2;
			}
			else if (type == 8 && angle < 0) {
				angle += M_PI * 2;
			}

			// check if the poses are reached
			for (int i = 0; i < angles.size(); i++) {
				if (visited[i]) continue;

				if (angles[2] >= angles[0]) {
					if (angle >= angles[i]) {
						visited[i] = true;
						unvisited--;
					}
				}
				else {
					if (angle <= angles[i]) {
						visited[i] = true;
						unvisited--;
					}
				}
			}

			// if all the poses are reached without collision, no collision is detected.
			if (unvisited == 0) {
				kinematics.clear();
				return false;
			}
		}

		kinematics.clear();
		return false;
	}

	double LinkageSynthesisRRRP::tortuosityOfTrajectory(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& points, const Object25D& moving_body) {
		// calculate the local coordinates of the body points
		glm::dmat3x3 inv_pose0 = glm::inverse(poses[0]);
		std::vector<glm::dvec2> body_pts_local(moving_body.polygons[0].points.size());
		for (int i = 0; i < moving_body.polygons[0].points.size(); i++) {
			body_pts_local[i] = glm::dvec2(inv_pose0 * glm::dvec3(moving_body.polygons[0].points[i], 1));
		}

		// calculate the length of the motion using straight lines between poses
		double length_of_straight = 0.0;
		std::vector<glm::dvec2> prev_body_pts = moving_body.polygons[0].points;
		for (int i = 1; i < poses.size(); i++) {
			std::vector<glm::dvec2> next_body_pts(moving_body.polygons[0].points.size());
			for (int k = 0; k < moving_body.polygons[0].points.size(); k++) {
				next_body_pts[k] = glm::dvec2(poses[i] * glm::dvec3(body_pts_local[k], 1));
				length_of_straight += glm::length(next_body_pts[k] - prev_body_pts[k]);
			}
			prev_body_pts = next_body_pts;
		}

		// create a kinematics
		std::vector<glm::dvec2> connector_pts;
		kinematics::Kinematics kinematics = constructKinematics(poses, points, moving_body, false, {}, connector_pts);
		kinematics.diagram.initialize();

		// initialize the trajectory of the moving body
		prev_body_pts = moving_body.polygons[0].points;
		double length_of_trajectory = 0.0;

		// calculate the rotational angle of the driving crank for 1st, 2nd, and last poses
		// i.e., angles[0] = first pose, angles[1] = second pose, angles[2] = last pose
		std::vector<double> angles(3);
		glm::dvec2 w(glm::inverse(poses[0]) * glm::dvec3(points[2], 1));
		for (int i = 0; i < 2; i++) {
			glm::dvec2 W = glm::dvec2(poses[i] * glm::dvec3(w, 1));
			angles[i] = atan2(W.y - points[0].y, W.x - points[0].x);
		}
		{
			glm::dvec2 W = glm::dvec2(poses.back() * glm::dvec3(w, 1));
			angles[2] = atan2(W.y - points[0].y, W.x - points[0].x);
		}

		// order the angles based on their signs
		int type = 0;
		if (angles[0] < 0 && angles[1] < 0 && angles[2] >= 0 && angles[0] >= angles[1]) {
			type = 1;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] < 0 && angles[1] >= 0 && angles[2] < 0 && angles[0] < angles[2]) {
			type = 2;
			angles[1] -= M_PI * 2;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] < 0 && angles[1] >= 0 && angles[2] < 0 && angles[0] >= angles[2]) {
			type = 3;
			angles[2] += M_PI * 2;
		}
		else if (angles[0] < 0 && angles[1] >= 0 && angles[2] >= 0 && (poses.size() >= 3 && angles[1] >= angles[2] || poses.size() == 2 && angles[1] - angles[0] > M_PI)) {
			type = 4;
			angles[1] -= M_PI * 2;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] < 0 && angles[2] < 0 && (poses.size() >= 3 && angles[1] < angles[2] || poses.size() == 2 && angles[0] - angles[1] > M_PI)) {
			type = 5;
			angles[1] += M_PI * 2;
			angles[2] += M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] < 0 && angles[2] >= 0 && angles[0] < angles[2]) {
			type = 6;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] < 0 && angles[2] >= 0 && angles[0] >= angles[2]) {
			type = 7;
			angles[1] += M_PI * 2;
			angles[2] += M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] >= 0 && angles[2] < 0 && angles[0] < angles[1]) {
			type = 8;
			angles[2] += M_PI * 2;
		}

		if (angles[2] < angles[0]) {
			kinematics.invertSpeed();
		}

		// initialize the visited flag
		std::vector<bool> visited(angles.size(), false);
		visited[0] = true;
		int unvisited = 2;

		// run forward until collision is deteted or all the poses are reached
		while (true) {
			try {
				kinematics.stepForward(false, false);	// no collision check
			}
			catch (char* ex) {
				// if only some of the poses are reached before collision, the collision is detected.
				kinematics.clear();
				return length_of_trajectory / length_of_straight;
			}

			// calculate the angle of the driving crank
			double angle = atan2(kinematics.diagram.joints[2]->pos.y - points[0].y, kinematics.diagram.joints[2]->pos.x - points[0].x);

			// update the lengths of the trajectory of the moving body
			std::vector<glm::dvec2> next_body_pts = kinematics.diagram.bodies[0]->getActualPoints()[0];
			for (int i = 0; i < next_body_pts.size(); i++) {
				double length = glm::length(next_body_pts[i] - prev_body_pts[i]);
				length_of_trajectory += length;
				prev_body_pts[i] = next_body_pts[i];
			}

			// convert the sign of the angle
			if (type == 1 && angle > 0) {
				angle -= M_PI * 2;
			}
			else if (type == 2 && angle > angles[0]) {
				angle -= M_PI * 2;
			}
			else if (type == 3 && angle < angles[0]) {
				angle += M_PI * 2;
			}
			else if (type == 4 && angle > 0) {
				angle -= M_PI * 2;
			}
			else if (type == 5 && angle < 0) {
				angle += M_PI * 2;
			}
			else if (type == 6 && angle > angles[0]) {
				angle -= M_PI * 2;
			}
			else if (type == 7 && angle < angles[0]) {
				angle += M_PI * 2;
			}
			else if (type == 8 && angle < 0) {
				angle += M_PI * 2;
			}

			// check if the poses are reached
			for (int i = 0; i < angles.size(); i++) {
				if (visited[i]) continue;

				if (angles[2] >= angles[0]) {
					if (angle >= angles[i]) {
						visited[i] = true;
						unvisited--;
					}
				}
				else {
					if (angle <= angles[i]) {
						visited[i] = true;
						unvisited--;
					}
				}
			}

			// if all the poses are reached without collision, no collision is detected.
			if (unvisited == 0) {
				kinematics.clear();
				return length_of_trajectory / length_of_straight;
			}
		}

		kinematics.clear();
		return length_of_trajectory / length_of_straight;
	}

}