#include "dicp_se3.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

class dicp_node : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    dicp_se3 dicp_;
    int frame_id_ = 0;

    std::string doppler_mode;
    std::string file_path;
    std::string topic_name;
    double doppler_weight;
    double doppler_weight_max;
    double topic_hz;

public:
    dicp_node() : Node("dicp_node"), dicp_()
    {

        this->declare_parameter("file_path", "./dicp_path.txt");
        this->declare_parameter("topic_name", "/points");
        this->declare_parameter("topic_hz", 10.0);
        this->declare_parameter("doppler_mode", "adaptive");
        this->declare_parameter("doppler_weight", 0.01);
        this->declare_parameter("doppler_weight_max", 0.01);

        file_path = this->get_parameter("file_path").as_string();
        topic_name = this->get_parameter("topic_name").as_string();
        topic_hz = this->get_parameter("topic_hz").as_double();
        doppler_mode = this->get_parameter("doppler_mode").as_string();
        doppler_weight = this->get_parameter("doppler_weight").as_double();
        doppler_weight_max = this->get_parameter("doppler_weight_max").as_double();

        this->declare_parameter("use_extrinsic", true);
        this->declare_parameter("rebase_to_origin", true);

        this->declare_parameter("extrinsic.tx", 0.0);
        this->declare_parameter("extrinsic.ty", 0.0);
        this->declare_parameter("extrinsic.tz", 0.0);
        this->declare_parameter("extrinsic.qx", 0.0);
        this->declare_parameter("extrinsic.qy", 0.0);
        this->declare_parameter("extrinsic.qz", 0.0);
        this->declare_parameter("extrinsic.qw", 1.0);

        bool use_extrinsic_ = this->get_parameter("use_extrinsic").as_bool();
        bool rebase_to_origin_ = this->get_parameter("rebase_to_origin").as_bool();

        const double tx = this->get_parameter("extrinsic.tx").as_double();
        const double ty = this->get_parameter("extrinsic.ty").as_double();
        const double tz = this->get_parameter("extrinsic.tz").as_double();
        const double qx = this->get_parameter("extrinsic.qx").as_double();
        const double qy = this->get_parameter("extrinsic.qy").as_double();
        const double qz = this->get_parameter("extrinsic.qz").as_double();
        const double qw = this->get_parameter("extrinsic.qw").as_double();

        Eigen::Quaterniond q(qw, qx, qy, qz);
        q.normalize();

        Eigen::Matrix4d T_VS = Eigen::Matrix4d::Identity();
        T_VS.block<3,3>(0,0) = q.toRotationMatrix();
        T_VS(0,3) = tx;
        T_VS(1,3) = ty;
        T_VS(2,3) = tz;

        dicp_.SetDt(1.0 / topic_hz);

        std::ofstream(file_path, std::ios::trunc).close();

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic_name,
            rclcpp::SensorDataQoS(),
            std::bind(&dicp_node::PointCallBack, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "dicp node started");
    }

    ~dicp_node() = default;

private:
void PointCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    static float time = 0.0f;

    pcl::PointCloud<PointXYZD>::Ptr cloud(new pcl::PointCloud<PointXYZD>);
    pcl::fromROSMsg(*msg, *cloud);

    if (cloud->empty())
    {
        RCLCPP_WARN(this->get_logger(), "empty cloud");
        return;
    }
    // std::cout << "11111111111111" <<std::endl;

    pcl::PointCloud<PointXYZD>::Ptr cloud_filtered(new pcl::PointCloud<PointXYZD>);
    cloud_filtered = cloud;

    // pcl::VoxelGrid<PointXYZD> voxel;
    // voxel.setInputCloud(cloud);
    // voxel.setLeafSize(0.1f, 0.1f, 0.1f);
    // voxel.filter(*cloud_filtered);

    // std::cout << "11111111111111" <<std::endl;

    if (cloud_filtered->empty())
    {
        RCLCPP_WARN(this->get_logger(), "empty cloud after voxel filtering");
        return;
    }

    if (!dicp_.SetSource(cloud_filtered))
    {
        dicp_.AppendPoseToFile(time / topic_hz, file_path);
        RCLCPP_INFO(this->get_logger(), "first frame, initialize target only");
        return;
    }
    // std::cout << "11111111111111" <<std::endl;

    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    Sophus::SO3d rot = Sophus::SO3d();

    const int max_iter = 50;
    const double cost_eps = 1e-8;
    const double trans_eps = 1e-8;
    const double rot_eps = 1e-8;

    double last_cost = std::numeric_limits<double>::infinity();
    double curr_cost = 0.0;

    int used_iter = 0;

    for (int iter = 0; iter < max_iter; ++iter)
    {
        used_iter = iter + 1;

        std::vector<PointResidualJacobian> res;
        const double doppler_weight = GetDopplerWeight(iter);
        // std::cout << "11111111111111" <<std::endl;

        dicp_.ComputerRes(pos, rot, res, doppler_weight);
        // std::cout << "11111111111111" <<std::endl;

        if (res.size() < 20)
        {
            RCLCPP_WARN(this->get_logger(), "too few correspondences: %zu", res.size());
            break;
        }

        Eigen::Vector3d pos_prev = pos;
        Sophus::SO3d rot_prev = rot;

        curr_cost = 0.0;
        dicp_.Solve(pos, rot, res, curr_cost);

        Eigen::Vector3d dpos = pos - pos_prev;
        Eigen::Vector3d dtheta = (rot_prev.inverse() * rot).log();

        double cost_change = std::abs(last_cost - curr_cost);

        // RCLCPP_INFO(this->get_logger(),
        //     "iter %d | cost %.8f | dcost %.8f | dtrans %.8e | drot %.8e | res %zu",
        //     iter,
        //     curr_cost,
        //     cost_change,
        //     dpos.norm(),
        //     dtheta.norm(),
        //     res.size());

        if (cost_change < cost_eps ||
            (dpos.norm() < trans_eps && dtheta.norm() < rot_eps))
        {

            if (iter <= 10)
            {
                last_cost = curr_cost;
                continue;
            }

            else{
                RCLCPP_INFO(this->get_logger(),
                    "converged at iter %d | cost %.8f | dcost %.8f | dtrans %.8e | drot %.8e",
                    iter,
                    curr_cost,
                    cost_change,
                    dpos.norm(),
                    dtheta.norm());
                break;
            }
        }

        last_cost = curr_cost;
    }

    Eigen::Matrix4d T_ = dicp_.GetRelativeTransform(pos, rot);

    dicp_.TransformPose(T_);
    dicp_.AppendPoseToFile((++time) / topic_hz, file_path);
    dicp_.SetTarget();

    const auto& path = dicp_.GetPoses();
    const auto& Tw = path.back();

    RCLCPP_INFO(this->get_logger(),
        "frame %d raw: %zu filtered: %zu | iter: %d | local: %.3f %.3f %.3f | global: %.3f %.3f %.3f",
        frame_id_++,
        cloud->size(),
        cloud_filtered->size(),
        used_iter,
        T_(0,3), T_(1,3), T_(2,3),
        Tw(0,3), Tw(1,3), Tw(2,3));
}


float GetDopplerWeight(int iter) const
{
    if (doppler_mode == "off")
        return 0.0f;

    if (doppler_mode == "fixed")
        return static_cast<float>(doppler_weight);

    if (doppler_mode == "adaptive")
    {
        if (iter < 5)
            return 0.0f;

        float ratio = static_cast<float>(iter - 1) / 8.0f;
        if (ratio > 1.0f) ratio = 1.0f;
        return static_cast<float>(doppler_weight_max) * ratio;
    }

    return 0.0f;
}

};
