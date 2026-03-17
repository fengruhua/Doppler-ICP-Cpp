#pragma once

#include <iostream>
#include <atomic>
#include <vector>
#include <fstream>
#include <string>

#include "utils.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <omp.h>

struct PointResidualJacobian
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double icp_residual = 0.0;
    double doppler_residual = 0.0;
    Eigen::Matrix<double, 1, 6> icp_jacobian = Eigen::Matrix<double, 1, 6>::Zero();
    Eigen::Matrix<double, 1, 6> doppler_jacobian = Eigen::Matrix<double, 1, 6>::Zero();
    int is_right = 0;
};


struct idx_nor
{
    int idx = -1;
    Eigen::Vector3d nor = Eigen::Vector3d::Zero();
};

class dicp_se3
{
private:
    pcl::KdTreeFLANN<PointXYZD> kdtree_;

    pcl::PointCloud<PointXYZD>::Ptr source_;   // t_{k+1}
    pcl::PointCloud<PointXYZD>::Ptr target_;   // t_k

    std::vector<idx_nor> nor_;

    Sophus::SO3d rot_;
    Eigen::Vector3d pos_;

    std::atomic_bool init_flag;

    double dt_ = 0.1;
    Eigen::Vector3d t_VL_;
    Eigen::Matrix3d R_VL_;

    std::vector<Eigen::Matrix4d> pose_;

public:
    dicp_se3();
    ~dicp_se3() = default;

    void SetDt(double dt);

    void SetTarget();
    bool SetSource(pcl::PointCloud<PointXYZD>::Ptr source);
    void ComputerTargetNorm();

    void ComputerRes(Eigen::Vector3d &pos_,
                     Sophus::SO3d &rot_,
                     std::vector<PointResidualJacobian> &res_,
                     double doppler_weight);

    void Solve(Eigen::Vector3d &pos_,
               Sophus::SO3d &rot_,
               std::vector<PointResidualJacobian> &res_,
               double &cost);

    Eigen::Matrix4d GetRelativeTransform(const Eigen::Vector3d& pos,
                                         const Sophus::SO3d& rot) const;

    void TransformPose(const Eigen::Matrix4d& T);
    void AppendPoseToFile(double timestamp, const std::string& file_path);

    const std::vector<Eigen::Matrix4d>& GetPoses() const { return pose_; }
};
