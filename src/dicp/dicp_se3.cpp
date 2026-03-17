#include "dicp_se3.hpp"

dicp_se3::dicp_se3()
{
    this->target_.reset(new pcl::PointCloud<PointXYZD>);
    this->source_.reset(new pcl::PointCloud<PointXYZD>);

    this->pos_ = Eigen::Vector3d::Zero();
    this->rot_ = Sophus::SO3d();

    this->t_VL_ = {1.419999122619629, 0.23999977111816406, 1.369999885559082};
    this->R_VL_ = Eigen::Matrix3d::Identity();

    this->dt_ = 0.1;
}

void dicp_se3::SetTarget()
{
    this->target_ = this->source_;
    this->kdtree_.setInputCloud(this->target_);
    this->ComputerTargetNorm();
}

bool dicp_se3::SetSource(pcl::PointCloud<PointXYZD>::Ptr source)
{
    if (!source || source->empty())
        return false;

    if (!this->init_flag.load())
    {
        this->target_ = source;
        this->kdtree_.setInputCloud(this->target_);
        this->ComputerTargetNorm();
        this->init_flag.store(true);
        return false;   // 第一帧只初始化
    }

    this->source_ = source;
    return true;
}
void dicp_se3::ComputerTargetNorm()
{
    if (!this->target_ || this->target_->empty())
        return;

    size_t N = this->target_->points.size();
    this->nor_.resize(N);

    for (size_t i = 0; i < N; i++)
    {
        auto &pt = this->target_->points[i];

        std::vector<int> idx(5);
        std::vector<float> dis(5);

        this->nor_[i].idx = static_cast<int>(i);
        this->nor_[i].nor = Eigen::Vector3d::Zero();

        if (this->kdtree_.nearestKSearch(pt, 5, idx, dis) < 5)
            continue;

        bool valid = true;
        for (size_t k = 0; k < 5; k++)
        {
            if (dis[k] > 0.5f * 0.5f)
            {
                valid = false;
                break;
            }
        }
        if (!valid)
            continue;

        Eigen::Matrix<double, 3, 5> neigh;

        for (int k = 0; k < 5; k++)
        {
            neigh(0, k) = this->target_->points[idx[k]].x;
            neigh(1, k) = this->target_->points[idx[k]].y;
            neigh(2, k) = this->target_->points[idx[k]].z;
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
        normal.normalize();

        this->nor_[i].nor = normal;
    }
}

void dicp_se3::ComputerRes(Eigen::Vector3d &pos_,
                           Sophus::SO3d &rot_,
                           std::vector<PointResidualJacobian> &res_,
                           double doppler_weight)
{
    res_.clear();

    if (!this->source_ || !this->target_)
        return;

    if (this->source_->empty() || this->target_->empty())
        return;

    if (this->dt_ < 1e-6)
        return;

    Eigen::Vector3d u_t = - pos_ / this->dt_;           // vehicle linear velocity
    Eigen::Vector3d omega_t = - rot_.log() / this->dt_; // vehicle angular velocity

    size_t N = this->source_->points.size();

    res_.resize(N);

    for (size_t i = 0; i < N; i++)
    {
        auto &p = this->source_->points[i];

        Eigen::Vector3d pt(p.x, p.y, p.z);
        double range = pt.norm();
        if (range < 1e-6)
            continue;

        Eigen::Vector3d pt_trans = rot_.matrix() * pt + pos_;

        PointXYZD search_point_;
        search_point_.x = pt_trans.x();
        search_point_.y = pt_trans.y();
        search_point_.z = pt_trans.z();

        std::vector<int> idx(1);
        std::vector<float> dis(1);
        // std::cout << "22222222" <<std::endl;

        if (this->kdtree_.nearestKSearch(search_point_, 1, idx, dis) < 1)
            continue;
        // std::cout << "4444444444444444" <<std::endl;

        if (dis[0] > 0.1 * 0.1)
            continue;

        Eigen::Vector3d normal = this->nor_[idx[0]].nor;
        if (normal.norm() < 1e-8)
            continue;
        // std::cout << "555555555555" <<std::endl;

        Eigen::Vector3d q(this->target_->points[idx[0]].x,
                          this->target_->points[idx[0]].y,
                          this->target_->points[idx[0]].z);

        PointResidualJacobian res;

        // ---------------- ICP ----------------
        Eigen::Vector3d rot_jacobian = pt_trans.cross(normal);

        res.icp_residual = (1 - doppler_weight) * normal.dot(pt_trans - q);
        res.icp_jacobian(0, 0) = (1 - doppler_weight) * rot_jacobian.x();
        res.icp_jacobian(0, 1) = (1 - doppler_weight) * rot_jacobian.y();
        res.icp_jacobian(0, 2) = (1 - doppler_weight) * rot_jacobian.z();
        res.icp_jacobian(0, 3) = (1 - doppler_weight) * normal.x();
        res.icp_jacobian(0, 4) = (1 - doppler_weight) * normal.y();
        res.icp_jacobian(0, 5) = (1 - doppler_weight) * normal.z();

        // ---------------- Doppler ----------------
        // d_L = pt / ||pt||
        Eigen::Vector3d d_L = pt / range;

        // d_V = R_VL * d_L
        Eigen::Vector3d d_V = this->R_VL_ * d_L;

        // v_pred = d_V^T (u_t - omega_t x t_VL)
        double v_pred = - d_V.dot(u_t - omega_t.cross(this->t_VL_));

        // residual = meas - pred
        double r = p.doppler - v_pred;

        res.doppler_residual = doppler_weight * r;

        // J = (1/dt) [ (d_V x t_VL)^T , -d_V^T ]
        Eigen::Vector3d jr = d_V.cross(this->t_VL_);
        Eigen::Vector3d jt = - d_V;

        double inv_dt = 1.0 / this->dt_;

        res.doppler_jacobian(0, 0) = doppler_weight * inv_dt * jr.x();
        res.doppler_jacobian(0, 1) = doppler_weight * inv_dt * jr.y();
        res.doppler_jacobian(0, 2) = doppler_weight * inv_dt * jr.z();
        res.doppler_jacobian(0, 3) = doppler_weight * inv_dt * jt.x();
        res.doppler_jacobian(0, 4) = doppler_weight * inv_dt * jt.y();
        res.doppler_jacobian(0, 5) = doppler_weight * inv_dt * jt.z();
        res.is_right = 1;
        res_[i] = res;
    }
}

void dicp_se3::Solve(Eigen::Vector3d &pos_,
                     Sophus::SO3d &rot_,
                     std::vector<PointResidualJacobian> &res_,
                     double &cost)
{
    cost = 0.0;

    if (res_.empty())
        return;

    const double lambda = 1e-6;

    Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> g = Eigen::Matrix<double, 6, 1>::Zero();

    for (auto &res : res_)
    {
        if (res.is_right){
            {
                double r = res.icp_residual;
                Eigen::Matrix<double, 1, 6> J = res.icp_jacobian;

                H += J.transpose() * J;
                g += J.transpose() * r;
                cost += 0.5 * r * r;
            }

            {
                double r = res.doppler_residual;
                Eigen::Matrix<double, 1, 6> J = res.doppler_jacobian;

                H += J.transpose() * J;
                g += J.transpose() * r;
                cost += 0.5 * r * r;
            }
        }
    }

    Eigen::Matrix<double, 6, 6> H_lm = H;
    for (int k = 0; k < 6; ++k)
    {
        H_lm(k, k) += lambda * std::max(1e-12, H(k, k));
    }

    Eigen::Matrix<double, 6, 1> delta = -H_lm.ldlt().solve(g);

    if (!delta.allFinite())
        return;

    if (delta.norm() < 1e-6)
        return;

    Eigen::Vector3d dtheta = delta.head<3>();
    Eigen::Vector3d dt = delta.tail<3>();

    Sophus::SO3d dR = Sophus::SO3d::exp(dtheta);

    rot_ = dR * rot_;
    pos_ = dR * pos_ + dt;
}

Eigen::Matrix4d dicp_se3::GetRelativeTransform(const Eigen::Vector3d& pos,
                                               const Sophus::SO3d& rot) const
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = rot.matrix();
    T.block<3,1>(0,3) = pos;
    return T;
}

void dicp_se3::TransformPose(const Eigen::Matrix4d& T)
{
    if (pose_.empty())
    {
        pose_.push_back(Eigen::Matrix4d::Identity());
    }

    Eigen::Matrix4d T_odom = T.inverse();
    Eigen::Matrix4d global_pose = pose_.back() * T_odom;
    pose_.push_back(global_pose);
}

void dicp_se3::AppendPoseToFile(double timestamp, const std::string& file_path)
{
    if (pose_.empty())
        return;

    Eigen::Matrix4d T_out = pose_.back();

    Eigen::Quaterniond q(T_out.block<3,3>(0,0));
    q.normalize();

    std::ofstream ofs(file_path, std::ios::app);
    if (!ofs.is_open())
    {
        std::cerr << "Failed to open file: " << file_path << std::endl;
        return;
    }

    ofs << std::fixed << std::setprecision(6)
        << timestamp << " "
        << T_out(0,3) << " "
        << T_out(1,3) << " "
        << T_out(2,3) << " "
        << q.x() << " "
        << q.y() << " "
        << q.z() << " "
        << q.w() << "\n";
}

void dicp_se3::SetDt(double dt)
{
    if (dt > 1e-6)
        dt_ = dt;
}
