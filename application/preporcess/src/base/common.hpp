#pragma once
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "tools/tools_value_redefine.hpp"
// #include "tools/tools_logger_lite.hpp"

// extern loglevel_e loglevel;

using POINTTYPE = pcl::PointXYZI;
using POINTNORMALTYPE = pcl::PointXYZINormal;
using POINTCLOUD = pcl::PointCloud<POINTTYPE>;
using PointRGB = pcl::PointXYZRGB;


#define Lidar_FOV_Width 1280
#define Lidar_FOV_Height 960
#define Lidar_Focal_Length 800


namespace TypeDefs{

    using precision_t                   = double; 
    using Vector2Type                   = Eigen::Matrix<precision_t,2,1>;
    using Vector3Type                   = Eigen::Matrix<precision_t,3,1>;
    using Vector4Type                   = Eigen::Matrix<precision_t,4,1>;
    using Vector6Type                   = Eigen::Matrix<precision_t,6,1>;
    using DynamicVectorType             = Eigen::Matrix<precision_t,Eigen::Dynamic,1>;
    using QuaternionType                = Eigen::Quaternion<precision_t>;

    using Matrix3Type                   = Eigen::Matrix<precision_t,3,3>;
    using Matrix4Type                   = Eigen::Matrix<precision_t,4,4>;
    using Matrix6Type                   = Eigen::Matrix<precision_t,6,6>;
    using TransformType                 = Eigen::Isometry3d;//Matrix4Type;

    using TransformTypeList             = std::list<TransformType,Eigen::aligned_allocator<TransformType>>;
    using Vector2Vector                 = std::vector<Vector2Type,Eigen::aligned_allocator<Vector2Type>>;
    using Vector3Vector                 = std::vector<Vector3Type,Eigen::aligned_allocator<Vector3Type>>;
    using Vector4Vector                 = std::vector<Vector4Type,Eigen::aligned_allocator<Vector4Type>>;
    using Matrix3Vector                 = std::vector<Vector4Type,Eigen::aligned_allocator<Matrix3Type>>;
    using Matrix4Vector                 = std::vector<Vector4Type,Eigen::aligned_allocator<Matrix4Type>>;
};

// namespace pcl{
//     struct PointXYZ;
//     struct PointXYZINormal;

//     struct PointXYZRGB;
//     struct PointXYZ;

//     template <typename PointT>
//     class PointCloud{
//     //     inline Ptr
//     //   makeShared () const { return Ptr (new PointCloud<PointT> (*this)); }        
//     };    


// // template <typename PointT>
// // void pointcloud_size(typename pcl::PointCloud<PointT>::Ptr cloud)
// };
// using POINTTYPE = pcl::PointXYZ;
// using POINTNORMALTYPE = pcl::PointXYZINormal;
// using POINTCLOUD = pcl::PointCloud<POINTTYPE>;
// using PointRGB = pcl::PointXYZRGB;