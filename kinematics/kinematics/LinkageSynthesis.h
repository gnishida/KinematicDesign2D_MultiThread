#pragma once

#include <vector>
#include <glm/glm.hpp>
#include <opencv2/opencv.hpp>
#include "Solution.h"
#include "BBox.h"
#include "KinematicUtils.h"
#include "Kinematics.h"

namespace kinematics {

	class Particle {
	public:
		double cost;
		Solution solution;

	public:
		Particle() : cost(std::numeric_limits<double>::max()) {}
		Particle(double cost, const Solution& solution) : cost(cost), solution(solution) {}
	};

	class LinkageSynthesis {
	protected:
		std::vector<Object25D> fixed_bodies;
		std::vector<std::pair<double, double>> sigmas;
		bool rotatable_crank;
		bool avoid_branch_defect;
		double min_transmission_angle;
		double min_link_length;
		std::vector<double> weights;

	protected:
		LinkageSynthesis() {}

	public:
		static void calculateStatistics(const std::vector<double>& values, double& mean, double& sd);
		static bool compare(const Particle& s1, const Particle& s2);
		std::vector<glm::dmat3x3> perturbPoses(const std::vector<glm::dmat3x3>& poses, std::vector<std::pair<double, double>>& sigmas, double& position_error, double& orientation_error);
		void createDistanceMapForLinkageRegion(const std::vector<glm::dvec2>& linkage_region_pts, double scale, BBox& dist_map_bbox, cv::Mat& dist_map);
		void particleFilter(std::vector<Solution>& solutions, const std::vector<glm::dvec2>& linkage_region_pts, const cv::Mat& dist_map, const BBox& dist_map_bbox, const std::vector<glm::dvec2>& linkage_avoidance_pts, const Object25D& moving_body, int num_particles, int num_iterations, bool record_file);
		void particleFilterThread(std::vector<Particle>& particles, const std::vector<glm::dvec2>& linkage_region_pts, const BBox& bbox, const cv::Mat& dist_map, const BBox& dist_map_bbox, const std::vector<glm::dvec2>& linkage_avoidance_pts, const Object25D& moving_body);
		void resample(std::vector<Particle> particles, int N, std::vector<Particle>& resampled_particles, double max_cost);

		virtual void calculateSolution(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& linkage_region_pts, const std::vector<glm::dvec2>& linkage_avoidance_pts, int num_samples, const Object25D& moving_body, std::vector<Solution>& solution, std::vector<glm::dvec2>& enlarged_linkage_region_pts) = 0;
		virtual bool optimizeCandidate(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& linkage_region_pts, const BBox& bbox, std::vector<glm::dvec2>& points) = 0;
		virtual Solution findBestSolution(const std::vector<glm::dmat3x3>& poses, std::vector<Solution>& solutions, const std::vector<glm::dvec2>& linkage_region_pts, const cv::Mat& dist_map, const BBox& dist_map_bbox, const std::vector<glm::dvec2>& linkage_avoidance_pts, const Object25D& moving_body, int num_particles, int num_iterations, bool record_file) = 0;
		virtual bool checkHardConstraints(std::vector<glm::dvec2>& points, const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& linkage_region_pts, const std::vector<glm::dvec2>& linkage_avoidance_pts, const Object25D& moving_body) = 0;
		virtual int getType(const std::vector<glm::dvec2>& points) = 0;
		virtual bool checkOrderDefect(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& points) = 0;
		virtual bool checkBranchDefect(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& points) = 0;
		virtual bool checkCircuitDefect(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& points) = 0;
		virtual Kinematics constructKinematics(const std::vector<glm::dvec2>& points, const Object25D& moving_body, bool connect_joints, const std::vector<Object25D>& fixed_bodies, std::vector<glm::dvec2>& connected_pt) = 0;
		virtual void updateBodies(Kinematics& kinematics, const Object25D& moving_body) = 0;
		virtual double calculateCost(Solution& solution, const Object25D& moving_body, const cv::Mat& dist_map, const BBox& dist_map_bbox) = 0;
	};

}