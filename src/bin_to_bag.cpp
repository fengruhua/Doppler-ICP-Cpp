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
#include <rosbag2_cpp/writer.hpp>

#include <filesystem>
#include <fstream>
#include <vector>
#include <algorithm>
#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>

namespace fs = std::filesystem;

std::atomic<bool> g_stop(false);

void signalHandler(int signum)
{
    std::cout << "\nInterrupt signal (" << signum << ") received. Stopping...\n";
    g_stop = true;
}

struct PointXYZD {
    float x, y, z, doppler;
};

std::vector<PointXYZD> readBinFile(const std::string &filename)
{
    std::ifstream input(filename, std::ios::binary);
    input.seekg(0, std::ios::end);
    size_t num_bytes = input.tellg();
    input.seekg(0, std::ios::beg);

    size_t num_points = num_bytes / (4 * sizeof(float));
    std::vector<PointXYZD> points(num_points);
    input.read(reinterpret_cast<char *>(points.data()), num_bytes);

    return points;
}

sensor_msgs::msg::PointCloud2
convertToROSMsg(const std::vector<PointXYZD>& points, double timestamp_sec)
{
    sensor_msgs::msg::PointCloud2 msg;

    msg.header.stamp = rclcpp::Time(timestamp_sec * 1e9);
    msg.header.frame_id = "map";

    msg.height = 1;
    msg.width = points.size();
    msg.is_dense = false;
    msg.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2Fields(
        4,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "doppler", 1, sensor_msgs::msg::PointField::FLOAT32
    );

    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_i(msg, "doppler");

    for (size_t i = 0; i < points.size(); ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_i)
    {
        *iter_x = points[i].x;
        *iter_y = points[i].y;
        *iter_z = points[i].z;
        *iter_i = points[i].doppler;
    }

    return msg;
}

void show_progress(size_t current, size_t total) {
    const int bar_width = 50;
    float progress = static_cast<float>(current) / total;
    int pos = static_cast<int>(bar_width * progress);
    if ( current == 1 )
    {
        std::cout << "\r" << std::flush;
        return;
    }
    
    std::cout << "[" ;
    for (int i = 0; i < bar_width; ++i) {
        if (i < pos) std::cout << "=" ;
        else if (i == pos) std::cout << ">" ;
        else std::cout << " ";
    }
    // std::cout << "] " << int(progress * 100.0) << "%\r" << std::flush;
    std::cout << "] " << int(progress * 100.0) << std::endl;
}

int main(int argc, char **argv)
{
    if (argc < 3) {
        std::cout << "Usage:\n"
                  << argv[0]
                  << " <bin_folder> <output_bag_folder>\n";
        return -1;
    }

    rclcpp::init(argc, argv);

    std::string bin_folder = argv[1];
    std::string bag_folder = argv[2];

    // 注册 SIGINT 捕获
    std::signal(SIGINT, signalHandler);

    // 检查文件夹是否存在
    if (fs::exists(bag_folder)) {
        throw std::runtime_error(
            "Bag output directory already exists: " + bag_folder + 
            ". Please remove it or choose another folder."
        );
    }

    std::vector<std::string> file_list;

    for (auto &entry : fs::directory_iterator(bin_folder)) {
        if (entry.path().extension() == ".bin") {
            file_list.push_back(entry.path().string());
        }
    }

    std::sort(file_list.begin(), file_list.end());

    rosbag2_cpp::Writer writer;

    try {
        writer.open(bag_folder);
    } catch (const std::exception &e) {
        throw std::runtime_error(
            std::string("Failed to open bag folder: ") + bag_folder +
            ". Original error: " + e.what()
        );
    }

    writer.create_topic({
        "/doppler_points",
        "sensor_msgs/msg/PointCloud2",
        rmw_get_serialization_format(),
        ""
    });

    double dt = 0.1;
    size_t total_frames = file_list.size();

    for (size_t i = 0; i < total_frames; ++i)
    {
        if (g_stop) {  // Ctrl+C 检测
            std::cout << "\nEarly termination requested. Shutting down...\n";
            break;
        }

        auto points = readBinFile(file_list[i]);
        auto msg = convertToROSMsg(points, i * dt);

        writer.write(
            msg,
            "/doppler_points",
            rclcpp::Time(i * dt * 1e9)
        );
        show_progress(i + 1, total_frames);
    }

    std::cout << std::endl << "Bag file generated successfully." << std::endl;
    rclcpp::shutdown();

    return 0;
}