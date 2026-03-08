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

public:
    dicp_node() : Node("dicp_node"), dicp_(0.01)
    {

        std::ofstream("./dicp_path.txt", std::ios::trunc).close();

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/points",
            rclcpp::SensorDataQoS(),
            std::bind(&dicp_node::PointCallBack, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "dicp node started");
    }

    ~dicp_node() = default;
    // {
    //     dicp_.SavePath("/home/fengruhua/Doppler/Doppler-ICP-Cpp/dicp_path.txt");
    // }

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
            dicp_.AppendPoseToFile(time / 10, "./dicp_path.txt");
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

            float w = (iter < 2) ? 0.0f : 0.01f;
            dicp_.Slove(T_, icp_res, doppler_res, w);
        }

        dicp_.TransformPose(T_);
        dicp_.AppendPoseToFile((++time) / 10, "./dicp_path.txt");
        dicp_.UpdateTarget();

        const auto& path = dicp_.GetPath();
        const auto& Tw = path.back();

        RCLCPP_INFO(this->get_logger(),
            "frame %d local: %.3f %.3f %.3f | global: %.3f %.3f %.3f",
            frame_id_++,
            T_(0,3), T_(1,3), T_(2,3),
            Tw(0,3), Tw(1,3), Tw(2,3));
    }
};
