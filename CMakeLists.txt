cmake_minimum_required(VERSION 3.5)
project(doppler_icp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

add_compile_options(-O3 -march=native)

cmake_policy(SET CMP0074 NEW)

# ========================
# Find packages
# ========================

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Python3 COMPONENTS Development REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(glog REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(GTSAM REQUIRED)
find_package(rosbag2_cpp REQUIRED)

# ========================
# OpenMP
# ========================

if(OpenMP_CXX_FOUND)
    add_definitions(-DMP_EN=1)
    add_definitions(-DMP_PROC_NUM=4)
endif()

# ========================
# Common source (共享库部分)
# ========================

set(COMMON_SOURCES
    include/registration.cpp
    include/utils.cpp
)

# ========================
# 1 DICP executable
# ========================

add_executable(doppler_icp_node
    src/dicp_node.cpp
    ${COMMON_SOURCES}
)

# ========================
# 2 Dataset executable
# ========================

add_executable(dataset_node
    src/dataset_node.cpp
    ${COMMON_SOURCES}
)

# ========================
# 3 bin_to_bag executable
# ========================

add_executable(bin_to_bag_node
    src/bin_to_bag.cpp
    ${COMMON_SOURCES}
)

# ========================
# Include
# ========================

target_include_directories(doppler_icp_node PRIVATE
    include
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
)

target_include_directories(dataset_node PRIVATE
    include
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
)

# ========================
# ROS dependencies
# ========================

ament_target_dependencies(doppler_icp_node
    rclcpp
    sensor_msgs
    nav_msgs
    geometry_msgs
    tf2_ros
    pcl_conversions
)

ament_target_dependencies(dataset_node
    rclcpp
    sensor_msgs
)

ament_target_dependencies(bin_to_bag_node
  rclcpp
  sensor_msgs
  rosbag2_cpp
)

# ========================
# Link libraries
# ========================

target_link_libraries(doppler_icp_node
    ${PCL_LIBRARIES}
    yaml-cpp
    glog::glog
    gtsam
)

target_link_libraries(dataset_node
    ${PCL_LIBRARIES}
)

if(OpenMP_CXX_FOUND)
    target_link_libraries(doppler_icp_node OpenMP::OpenMP_CXX)
    target_link_libraries(dataset_node OpenMP::OpenMP_CXX)
endif()

# ========================
# Install
# ========================

install(TARGETS
    doppler_icp_node
    dataset_node
    bin_to_bag_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
    DESTINATION share/${PROJECT_NAME}
)

ament_package()