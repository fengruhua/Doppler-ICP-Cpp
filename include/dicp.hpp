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

    Eigen::Matrix4d T_VS_ = Eigen::Matrix4d::Identity();
    bool use_extrinsic_ = true;
    bool rebase_to_origin_ = true;
    double dt_ = 0.1;

    bool has_rebase_origin_ = false;
    Eigen::Matrix4d T_origin_inv_ = Eigen::Matrix4d::Identity();

public:
    dicp();
    ~dicp() = default;

    void SetExtrinsic(const Eigen::Matrix4d& T_VS);
    void SetUseExtrinsic(bool use_extrinsic);
    void SetRebaseToOrigin(bool rebase_to_origin);
    void SetDt(double dt);

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
    void AppendPoseToFile(double timestamp, const std::string& file_path);
    const std::vector<Eigen::Matrix4d>& GetPath() const { return pose_; }
};
