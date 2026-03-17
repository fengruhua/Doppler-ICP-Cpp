#pragma once
#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>
struct EIGEN_ALIGN16 PointXYZD
{
    PCL_ADD_POINT4D;
    float doppler;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZD,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, doppler, doppler)
)
