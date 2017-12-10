#include "LinkageSynthesis.h"
#include <QFile>
#include <QTextStream>
#include <boost/thread.hpp>

namespace kinematics {

	void LinkageSynthesis::calculateStatistics(const std::vector<double>& values, double& mean, double& sd) {
		double total = 0.0;
		for (int i = 0; i < values.size(); i++) {
			total += values[i];
		}
		mean = total / values.size();

		double var = 0.0;
		for (int i = 0; i < values.size(); i++) {
			var += (values[i] - mean) * (values[i] - mean);
		}
		sd = std::sqrt(var / values.size());
	}

	bool LinkageSynthesis::compare(const Particle& s1, const Particle& s2) {
		return s1.cost < s2.cost;
	}

	/**
	 * Perturbe the poses a little based on the sigma.
	 */
	std::vector<glm::dmat3x3> LinkageSynthesis::perturbPoses(const std::vector<glm::dmat3x3>& poses, std::vector<std::pair<double, double>>& sigmas, double& position_error, double& orientation_error) {
		std::vector<glm::dmat3x3> perturbed_poses = poses;

		position_error = 0.0;
		orientation_error = 0.0;

		for (int i = 0; i < perturbed_poses.size(); i++) {
			double e1 = 0;
			double e2 = 0;
			double delta_theta = 0;
			if (i == 0) {	// first pose
				e1 = genNormal(0, sigmas[0].first);
				e2 = genNormal(0, sigmas[0].first);
				delta_theta = genNormal(0, sigmas[0].second);
			}
			else if (i == perturbed_poses.size() - 1) {	// last pose
				e1 = genNormal(0, sigmas[2].first);
				e2 = genNormal(0, sigmas[2].first);
				delta_theta = genNormal(0, sigmas[2].second);
			}
			else {	// poses in the middle
				e1 = genNormal(0, sigmas[1].first);
				e2 = genNormal(0, sigmas[1].first);
				delta_theta = genNormal(0, sigmas[1].second);
			}

			perturbed_poses[i][2][0] += e1;
			perturbed_poses[i][2][1] += e2;
			position_error += std::sqrt(e1 * e1 + e2 * e2);

			double theta = atan2(poses[i][0][1], poses[i][0][0]) + delta_theta;
			perturbed_poses[i][0][0] = cos(theta);
			perturbed_poses[i][0][1] = sin(theta);
			perturbed_poses[i][1][0] = -sin(theta);
			perturbed_poses[i][1][1] = cos(theta);
			orientation_error += abs(delta_theta);
		}

		return perturbed_poses;
	}

	/**
	 * Create a distance map for the linkage region.
	 */
	void LinkageSynthesis::createDistanceMapForLinkageRegion(const std::vector<glm::dvec2>& linkage_region_pts, double scale, BBox& dist_map_bbox, cv::Mat& dist_map) {
		// calculate the center of the linkage region
		BBox bbox = boundingBox(linkage_region_pts);
		glm::dvec2 center = bbox.center();

		// calculate the enlarged linkage region for the sampling region
		std::vector<glm::dvec2> enlarged_linkage_region_pts;
		for (int i = 0; i < linkage_region_pts.size(); i++) {
			enlarged_linkage_region_pts.push_back((linkage_region_pts[i] - center) * (double)scale + center);
		}

		// calculate the bounding box for the distance map
		dist_map_bbox = boundingBox(enlarged_linkage_region_pts);

		cv::Mat img(dist_map_bbox.height() + 1, dist_map_bbox.width() + 1, CV_8U, cv::Scalar(255));

		std::vector<std::vector<cv::Point>> pts(1);
		for (int i = 0; i < linkage_region_pts.size(); i++) {
			double x = linkage_region_pts[i].x - dist_map_bbox.minPt.x;
			double y = linkage_region_pts[i].y - dist_map_bbox.minPt.y;
			pts[0].push_back(cv::Point(x, y));
		}
		cv::fillPoly(img, pts, cv::Scalar(0), 4);
		
		cv::distanceTransform(img, dist_map, CV_DIST_L2, 3);
		//cv::imwrite("test2.png", img);
		//cv::imwrite("test.png", dist_map);

		// convert float type to double type
		dist_map.convertTo(dist_map, CV_64F);
	}

	void LinkageSynthesis::particleFilter(std::vector<Solution>& solutions, const std::vector<glm::dvec2>& linkage_region_pts, const cv::Mat& dist_map, const BBox& dist_map_bbox, const std::vector<glm::dvec2>& linkage_avoidance_pts, const Object25D& moving_body, int num_particles, int num_iterations, bool record_file) {
		BBox linkage_region_bbox = boundingBox(linkage_region_pts);

		/*
		std::vector<Particle> particles(solutions.size());

		double max_cost = 0;
		for (int i = 0; i < solutions.size(); i++) {
			double cost = calculateCost(solutions[i], moving_body, dist_map, dist_map_bbox, weights);
			max_cost = std::max(max_cost, cost);
			particles[i] = Particle(cost, solutions[i]);
		}
		*/

		// augment
		double max_cost = 0;
		std::vector<Particle> particles(num_particles);
		for (int i = 0; i < num_particles; i++) {
			double cost = calculateCost(solutions[i % solutions.size()], moving_body, dist_map, dist_map_bbox);
			max_cost = std::max(max_cost, cost);
			particles[i] = Particle(cost, solutions[i % solutions.size()]);
		}

		QFile* file;
		QTextStream* out;

		if (record_file) {
			file = new QFile("particle_filter.txt");
			file->open(QIODevice::WriteOnly);
			out = new QTextStream(file);

			std::vector<double> values;
			for (int i = 0; i < particles.size(); i++) {
				if (particles[i].cost == std::numeric_limits<double>::max()) continue;

				values.push_back(particles[i].cost);
			}
			double mean_val;
			double sd_val;
			calculateStatistics(values, mean_val, sd_val);
			(*out) << mean_val << "," << sd_val << "\n";
		}

		// particle filter
		for (int iter = 0; iter < num_iterations; iter++) {
			const int NUM_THREADS = 8;
			std::vector<boost::thread> threads(NUM_THREADS);
			std::vector<std::vector<Particle>> new_particles(NUM_THREADS);
			for (int i = 0; i < threads.size(); i++) {
				int offset1 = i * particles.size() / NUM_THREADS;
				int offset2 = (i + 1) * particles.size() / NUM_THREADS;
				new_particles[i] = std::vector<Particle>(particles.begin() + offset1, particles.begin() + offset2);
				threads[i] = boost::thread(&LinkageSynthesis::particleFilterThread, this, boost::ref(new_particles[i]), boost::ref(linkage_region_pts), boost::ref(linkage_region_bbox), boost::ref(dist_map), boost::ref(dist_map_bbox), boost::ref(linkage_avoidance_pts), boost::ref(moving_body));
			}
			for (int i = 0; i < threads.size(); i++) {
				threads[i].join();
			}




			// merge the particles
			for (int i = 0; i < threads.size(); i++) {
				particles.insert(particles.end(), new_particles[i].begin(), new_particles[i].end());
			}

			// resampling
			resample(particles, num_particles, particles, max_cost);

			if (record_file) {
				std::vector<double> values;
				for (int i = 0; i < particles.size(); i++) {
					if (particles[i].cost == std::numeric_limits<double>::max()) continue;

					values.push_back(particles[i].cost);
				}
				double mean_val;
				double sd_val;
				calculateStatistics(values, mean_val, sd_val);
				(*out) << mean_val << "," << sd_val << "\n";
			}
		}

		if (record_file) {
			file->close();
			delete out;
			delete file;
		}

		// sort the particles based on the costs
		sort(particles.begin(), particles.end(), compare);

		// update solutions
		solutions.resize(particles.size());
		for (int i = 0; i < particles.size(); i++) {
			solutions[i] = particles[i].solution;
		}
	}

	void LinkageSynthesis::particleFilterThread(std::vector<Particle>& particles, const std::vector<glm::dvec2>& linkage_region_pts, const BBox& bbox, const cv::Mat& dist_map, const BBox& dist_map_bbox, const std::vector<glm::dvec2>& linkage_avoidance_pts, const Object25D& moving_body) {
		// perturb the particles and calculate its score
		for (int i = 0; i < particles.size(); i++) {
			// pertube the joints
			for (int j = 0; j < particles[i].solution.points.size(); j++) {
				particles[i].solution.points[j].x += genRand(-1, 1);
				particles[i].solution.points[j].y += genRand(-1, 1);
			}

			if (optimizeCandidate(particles[i].solution.poses, linkage_region_pts, bbox, particles[i].solution.points)) {
				// check the hard constraints
				if (checkHardConstraints(particles[i].solution.points, particles[i].solution.poses, linkage_region_pts, linkage_avoidance_pts, moving_body)) {
					// calculate the score
					double cost = calculateCost(particles[i].solution, moving_body, dist_map, dist_map_bbox);
					particles[i] = Particle(cost, particles[i].solution);
				}
				else {
					// for the invalid point, make the cost infinity so that it will be discarded.
					particles[i] = Particle(std::numeric_limits<double>::max(), particles[i].solution);
				}
			}
			else {
				// for the invalid point, make the cost infinity so that it will be discarded.
				particles[i] = Particle(std::numeric_limits<double>::max(), particles[i].solution);
			}
		}
	}

	void LinkageSynthesis::resample(std::vector<Particle> particles, int N, std::vector<Particle>& resampled_particles, double max_cost) {
		// calculate the weights of particles
		std::vector<double> weights(particles.size());
		double weight_total = 0.0;
		for (int i = 0; i < particles.size(); i++) {
			double w;
			if (particles[i].cost == std::numeric_limits<double>::max()) {
				w = 0;
			}
			else {
				w = std::exp(-particles[i].cost / max_cost * 20);
			}
			if (i == 0) {
				weights[i] = w;
			}
			else {
				weights[i] = weights[i - 1] + w;
			}
			weight_total += w;
		}
		for (int i = 0; i < particles.size(); i++) {
			weights[i] /= weight_total;
		}

		// resample the particles based on their weights
		resampled_particles.resize(N);
		for (int i = 0; i < N; i++) {
			double r = genRand();
			auto it = std::lower_bound(weights.begin(), weights.end(), r);
			int index = it - weights.begin();
			resampled_particles[i] = particles[index];
		}
	}

}