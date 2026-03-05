//  MIT License
// 
//  Copyright (c) 2026 fengruhua
// 
//  Permission is hereby granted, free of charge, to any person obtaining a copy
//  of this software and associated documentation files (the "Software"), to deal
//  in the Software without restriction, including without limitation the rights
//  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//  copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
// 
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
// 
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//  SOFTWARE.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <filesystem>
#include <fstream>
#include <vector>
#include <algorithm>
#include <iostream>
#include <csignal>
#include <atomic>

namespace fs = std::filesystem;

std::atomic<bool> g_stop(false);

void signalHandler(int signum)
{
    std::cout << "\nInterrupt signal (" << signum << ") received. Stopping...\n";
    g_stop = true;
}

struct PointXYZD
{
    float x;
    float y;
    float z;
    float doppler;
};

class DatasetPublisher : public rclcpp::Node
{

private:

    std::string data_path_;
    std::string point_topic_;
    std::string frame_id_;

    double frame_rate_;
    bool loop_;
    bool paused_;

    size_t current_index_;

    std::vector<std::string> file_list_;
    std::vector<sensor_msgs::msg::PointCloud2> messages_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

public:

    DatasetPublisher()
    : Node("dataset_publisher"),
      current_index_(0),
      paused_(false)
    {

        declare_parameter<std::string>("data_path","");
        declare_parameter<std::string>("point_topic","/points");
        declare_parameter<std::string>("frame_id","map");
        declare_parameter<double>("frame_rate",10.0);
        declare_parameter<bool>("loop",false);

        get_parameter("data_path",data_path_);
        get_parameter("point_topic",point_topic_);
        get_parameter("frame_id",frame_id_);
        get_parameter("frame_rate",frame_rate_);
        get_parameter("loop",loop_);

        if(data_path_.empty())
        {
            RCLCPP_FATAL(get_logger(),"data_path parameter not set");
            rclcpp::shutdown();
            return;
        }

        loadFileList();

        preloadDataset();

        publisher_=create_publisher<sensor_msgs::msg::PointCloud2>(
            point_topic_,10);

        timer_=create_wall_timer(
            std::chrono::milliseconds((int)(1000.0/frame_rate_)),
            std::bind(&DatasetPublisher::timerCallback,this));

        RCLCPP_INFO(get_logger(),
            "Dataset ready. Frames: %ld  Rate: %.2f Hz",
            messages_.size(),
            frame_rate_);
    }

private:

    void loadFileList()
    {

        if(!fs::exists(data_path_))
        {
            RCLCPP_FATAL(get_logger(),
                "Dataset path not found: %s",
                data_path_.c_str());
            rclcpp::shutdown();
            return;
        }

        for(auto &entry:fs::directory_iterator(data_path_))
        {
            if(entry.path().extension()==".bin")
            {
                file_list_.push_back(entry.path().string());
            }
        }

        std::sort(file_list_.begin(),file_list_.end());

        if(file_list_.empty())
        {
            RCLCPP_FATAL(get_logger(),"No bin files found.");
            rclcpp::shutdown();
        }
    }

    sensor_msgs::msg::PointCloud2 convertToMsg(const std::vector<PointXYZD>& points)
    {

        sensor_msgs::msg::PointCloud2 msg;

        msg.header.frame_id=frame_id_;
        msg.height=1;
        msg.width=points.size();
        msg.is_dense=false;
        msg.is_bigendian=false;

        sensor_msgs::PointCloud2Modifier modifier(msg);

        modifier.setPointCloud2Fields(
            4,
            "x",1,sensor_msgs::msg::PointField::FLOAT32,
            "y",1,sensor_msgs::msg::PointField::FLOAT32,
            "z",1,sensor_msgs::msg::PointField::FLOAT32,
            "intensity",1,sensor_msgs::msg::PointField::FLOAT32
        );

        modifier.resize(points.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(msg,"x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(msg,"y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(msg,"z");
        sensor_msgs::PointCloud2Iterator<float> iter_i(msg,"intensity");

        for(size_t i=0;i<points.size();++i,++iter_x,++iter_y,++iter_z,++iter_i)
        {
            *iter_x=points[i].x;
            *iter_y=points[i].y;
            *iter_z=points[i].z;
            *iter_i=points[i].doppler;
        }

        return msg;
    }

    void preloadDataset()
    {

        RCLCPP_INFO(get_logger(),"Preloading dataset...");

        for(size_t i=0;i<file_list_.size();++i)
        {

            std::ifstream input(file_list_[i],std::ios::binary);

            input.seekg(0,std::ios::end);
            size_t num_bytes=input.tellg();
            input.seekg(0,std::ios::beg);

            size_t num_points=num_bytes/(4*sizeof(float));

            std::vector<PointXYZD> points(num_points);

            input.read(reinterpret_cast<char*>(points.data()),num_bytes);

            auto msg=convertToMsg(points);

            messages_.push_back(msg);

            if(i%100==0)
            {
                std::cout<<"\rLoading "<<i<<"/"<<file_list_.size()<<std::flush;
            }
        }

        std::cout<<"\rLoading complete: "<<messages_.size()<<" frames"<<std::endl;
    }

    void timerCallback()
    {

        if (g_stop) {  // Ctrl+C 检测
            std::cout << "\nEarly termination requested. Shutting down...\n";
            return;
        }

        if(paused_)
            return;

        if(current_index_>=messages_.size())
        {

            if(loop_)
            {
                current_index_=0;
                RCLCPP_INFO(get_logger(),"Loop restart.");
                return;
            }

            RCLCPP_INFO(get_logger(),"Dataset finished.");
            g_stop.store(true);
            timer_->cancel();
            rclcpp::shutdown();

            return;
        }

        auto msg=messages_[current_index_];

        msg.header.stamp=this->now();

        publisher_->publish(msg);

        RCLCPP_INFO(
            this->get_logger(), 
            "Frame %ld/%ld : %s", 
            current_index_, 
            file_list_.size(), 
            file_list_[current_index_].c_str()
        );

        current_index_++;
    }

};

int main(int argc,char **argv)
{

    rclcpp::init(argc,argv);

    auto node=std::make_shared<DatasetPublisher>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}