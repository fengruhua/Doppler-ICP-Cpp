#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#include "dicp.hpp"

class dicp_node : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    dicp dicp_;
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

        dicp_.SetExtrinsic(T_VS);
        dicp_.SetUseExtrinsic(use_extrinsic_);
        dicp_.SetRebaseToOrigin(rebase_to_origin_);
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
        static float time = 0.0;
        pcl::PointCloud<PointXYZD>::Ptr cloud(new pcl::PointCloud<PointXYZD>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "empty cloud");
            return;
        }

        if (!dicp_.SetSource(cloud))
        {
            dicp_.AppendPoseToFile(time / topic_hz, file_path);
            RCLCPP_INFO(this->get_logger(), "first frame, initialize target only");
            return;
        }

        Eigen::Matrix4d T_ = Eigen::Matrix4d::Identity();

        for (int iter = 0; iter < 10; iter++)
        {
            std::vector<PointResidualJacobian> icp_res;
            std::vector<PointResidualJacobian> doppler_res;

            dicp_.ComputerPointToPlane(T_, icp_res);
            dicp_.ComputerDoppler(T_, doppler_res);

            if (icp_res.size() < 20)
            {
                RCLCPP_WARN(this->get_logger(), "too few correspondences: %zu", icp_res.size());
                break;
            }

            const float w = GetDopplerWeight(iter);
            dicp_.Slove(T_, icp_res, doppler_res, w);
        }

        dicp_.TransformPose(T_);
        dicp_.AppendPoseToFile((++time) / topic_hz, file_path);
        dicp_.UpdateTarget();

        const auto& path = dicp_.GetPath();
        const auto& Tw = path.back();

        RCLCPP_INFO(this->get_logger(),
            "frame %d local: %.3f %.3f %.3f | global: %.3f %.3f %.3f",
            frame_id_++,
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
            if (iter < 2)
                return 0.0f;

            float ratio = static_cast<float>(iter - 1) / 8.0f;
            if (ratio > 1.0f) ratio = 1.0f;
            return static_cast<float>(doppler_weight_max) * ratio;
        }

        return 0.0f;
    }

};
