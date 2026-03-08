#pragma once

#include <iostream>
#include <atomic>
#include <vector>
#include <fstream>

#include "utils.hpp"

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <omp.h>

struct PointResidualJacobian {
    double residual;
    Eigen::Matrix<double, 1, 6> jacobian;
};

class dicp
{
private:
    pcl::KdTreeFLANN<PointXYZD> kdtree_;

    pcl::PointCloud<PointXYZD>::Ptr source_; // t_t+1
    pcl::PointCloud<PointXYZD>::Ptr target_; // t_t

    std::vector<Eigen::Matrix4d> pose_;

    std::atomic_bool init_flag;
    float doppler_weight;

public:
    dicp(float doppler_weight = 0.5);
    ~dicp() = default;

    bool SetSource(const pcl::PointCloud<PointXYZD>::Ptr source);
    void UpdateTarget();

    void ComputerPointToPlane(const Eigen::Matrix4d& T,
                              std::vector<PointResidualJacobian>& icp_res);

    void ComputerDoppler(const Eigen::Matrix4d& T,
                         std::vector<PointResidualJacobian>& doppler_res);

    void Slove(Eigen::Matrix4d &T,
               std::vector<PointResidualJacobian>& icp_res,
               std::vector<PointResidualJacobian>& doppler_res,
               float doppler_weight);

    void TransformPose(const Eigen::Matrix4d& T);
    void AppendPoseToFile(double timestamp, const std::string& file_path) const;
    const std::vector<Eigen::Matrix4d>& GetPath() const { return pose_; }
};
