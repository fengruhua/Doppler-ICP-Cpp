#include "dicp.hpp"
#include <fstream>
#include <iomanip>

dicp::dicp(float doppler_weight)
{
    this->source_.reset(new pcl::PointCloud<PointXYZD>);
    this->target_.reset(new pcl::PointCloud<PointXYZD>);

    this->doppler_weight = doppler_weight;
    this->init_flag.store(false);
}

bool dicp::SetSource(const pcl::PointCloud<PointXYZD>::Ptr source)
{
    if (!this->init_flag.load())
    {
        this->target_ = source;
        this->kdtree_.setInputCloud(this->target_);
        this->init_flag.store(true);
        this->pose_.clear();
        this->pose_.push_back(Eigen::Matrix4d::Identity());
        return false;   // 第一帧，只初始化，不配准
    }

    this->source_ = source;
    return true;
}

void dicp::UpdateTarget()
{
    this->target_ = this->source_;
    this->kdtree_.setInputCloud(this->target_);
}

void dicp::ComputerPointToPlane(const Eigen::Matrix4d &T,
                                std::vector<PointResidualJacobian> &icp_res)
{
    if (!this->source_ || !this->target_ || this->source_->empty() || this->target_->empty())
        return;

    Eigen::Matrix3d r = T.block<3,3>(0,0);
    Eigen::Vector3d t = T.block<3,1>(0,3);

    size_t N = this->source_->points.size();

    int num_threads = omp_get_max_threads();
    std::vector<std::vector<PointResidualJacobian>> local_res(num_threads);

#pragma omp parallel for
    for (long long i = 0; i < static_cast<long long>(N); i++)
    {
        int tid = omp_get_thread_num();

        const auto &p = this->source_->points[i];
        Eigen::Vector3d pt(p.x, p.y, p.z);
        Eigen::Vector3d pt_trans = r * pt + t;

        PointXYZD search_point;
        search_point.x = static_cast<float>(pt_trans.x());
        search_point.y = static_cast<float>(pt_trans.y());
        search_point.z = static_cast<float>(pt_trans.z());

        std::vector<int> idx(5);
        std::vector<float> dis(5);

        if (this->kdtree_.nearestKSearch(search_point, 5, idx, dis) < 5)
            continue;

        if (dis[4] > 0.3f * 0.3f)
            continue;

        Eigen::Matrix<double, 3, 5> neigh;
        for (int k = 0; k < 5; k++)
        {
            neigh(0,k) = this->target_->points[idx[k]].x;
            neigh(1,k) = this->target_->points[idx[k]].y;
            neigh(2,k) = this->target_->points[idx[k]].z;
        }

        Eigen::Vector3d centroid = neigh.rowwise().mean();

        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
        for (int k = 0; k < 5; k++)
        {
            Eigen::Vector3d d = neigh.col(k) - centroid;
            cov += d * d.transpose();
        }
        cov /= 5.0;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
        auto eval = solver.eigenvalues();

        if (eval(0) > eval(2) * 0.1)
            continue;

        Eigen::Vector3d normal = solver.eigenvectors().col(0);
        if (normal.dot(pt_trans - centroid) < 0)
            normal = -normal;

        PointResidualJacobian res;
        res.residual = normal.dot(pt_trans - centroid);

        Eigen::Vector3d rot_jacobian = pt_trans.cross(normal);

        res.jacobian(0, 0) = rot_jacobian.x();
        res.jacobian(0, 1) = rot_jacobian.y();
        res.jacobian(0, 2) = rot_jacobian.z();
        res.jacobian(0, 3) = normal.x();
        res.jacobian(0, 4) = normal.y();
        res.jacobian(0, 5) = normal.z();

        local_res[tid].push_back(res);
    }

    icp_res.clear();
    for (auto& vec : local_res)
    {
        icp_res.insert(icp_res.end(), vec.begin(), vec.end());
    }
}

void dicp::ComputerDoppler(const Eigen::Matrix4d &T,
                           std::vector<PointResidualJacobian> &doppler_res)
{
    if (!this->source_ || this->source_->empty())
        return;

    Eigen::Vector3d t = T.block<3,1>(0,3);
    Eigen::Vector3d u_t = t / 0.1;

    Eigen::Matrix3d R = T.block<3,3>(0,0);
    Eigen::AngleAxisd aa(R);
    Eigen::Vector3d w_t = aa.axis() * aa.angle() / 0.1;

    size_t N = this->source_->points.size();

    Eigen::Vector3d trans(1.4199991226,
                          0.2399997711,
                          1.3699998855);

    int num_threads = omp_get_max_threads();
    std::vector<std::vector<PointResidualJacobian>> local_res(num_threads);

#pragma omp parallel for
    for (long long i = 0; i < static_cast<long long>(N); i++)
    {
        int tid = omp_get_thread_num();

        const auto &p = source_->points[i];
        Eigen::Vector3d pt(p.x, p.y, p.z);

        if (pt.norm() < 1e-6)
            continue;

        Eigen::Vector3d d = pt.normalized();

        double v_pred = d.dot(u_t - w_t.cross(trans));

        PointResidualJacobian res;
        res.residual = p.doppler - v_pred;

        Eigen::Vector3d rot_j = -(trans.cross(d)) / 0.1;
        Eigen::Vector3d trans_j = -d / 0.1;

        res.jacobian <<
            rot_j.x(),
            rot_j.y(),
            rot_j.z(),
            trans_j.x(),
            trans_j.y(),
            trans_j.z();

        local_res[tid].push_back(res);
    }

    doppler_res.clear();
    for (auto& vec : local_res)
    {
        doppler_res.insert(doppler_res.end(), vec.begin(), vec.end());
    }
}

void dicp::Slove(Eigen::Matrix4d &T,
                 std::vector<PointResidualJacobian> &icp_res,
                 std::vector<PointResidualJacobian> &doppler_res,
                 float doppler_weight)
{
    if (icp_res.empty() && doppler_res.empty())
        return;

    const double lambda = 1e-6;

    Eigen::Matrix<double,6,6> H = Eigen::Matrix<double,6,6>::Zero();
    Eigen::Matrix<double,6,1> b = Eigen::Matrix<double,6,1>::Zero();

    for (const auto &res : icp_res)
    {
        H += res.jacobian.transpose() * res.jacobian;
        b += res.jacobian.transpose() * res.residual;
    }

    for (const auto &res : doppler_res)
    {
        H += doppler_weight * res.jacobian.transpose() * res.jacobian;
        b += doppler_weight * res.jacobian.transpose() * res.residual;
    }

    H += lambda * Eigen::Matrix<double,6,6>::Identity();

    Eigen::Matrix<double,6,1> dx = -H.ldlt().solve(b);

    if (!dx.allFinite())
        return;

    Eigen::Vector3d dtheta = dx.head<3>();
    Eigen::Vector3d dt = dx.tail<3>();

    // 限制单步，防止直接炸
    if (dtheta.norm() > 0.2) dtheta *= (0.2 / dtheta.norm());
    if (dt.norm() > 1.0) dt *= (1.0 / dt.norm());

    Eigen::Matrix3d dR = Eigen::Matrix3d::Identity();
    double angle = dtheta.norm();
    if (angle > 1e-12)
        dR = Eigen::AngleAxisd(angle, dtheta / angle).toRotationMatrix();

    Eigen::Matrix4d dT = Eigen::Matrix4d::Identity();
    dT.block<3,3>(0,0) = dR;
    dT.block<3,1>(0,3) = dt;

    T = dT * T;
}

void dicp::TransformPose(const Eigen::Matrix4d& T)
{
    if (pose_.empty())
    {
        pose_.push_back(Eigen::Matrix4d::Identity());
    }

    Eigen::Matrix4d T_odom = T.inverse();
    Eigen::Matrix4d global_pose = pose_.back() * T_odom;
    pose_.push_back(global_pose);
}

void dicp::AppendPoseToFile(double timestamp, const std::string& file_path) const
{
    if (pose_.empty())
        return;

    const auto& T = pose_.back();
    Eigen::Quaterniond q(T.block<3,3>(0,0));
    q.normalize();

    std::ofstream ofs(file_path, std::ios::app);
    if (!ofs.is_open())
    {
        std::cerr << "Failed to open file: " << file_path << std::endl;
        return;
    }

    ofs << std::fixed << std::setprecision(6)
        << timestamp << " "
        << T(0,3) << " "
        << T(1,3) << " "
        << T(2,3) << " "
        << q.x() << " "
        << q.y() << " "
        << q.z() << " "
        << q.w() << "\n";
}
