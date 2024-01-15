#pragma once
#include <iostream>

#include <pcl/io/pcd_io.h>

#include "tools/tools_value_redefine.hpp"
// #include "tools/tools_logger_lite.hpp"

// extern loglevel_e loglevel;

using POINTTYPE = pcl::PointXYZ;
using POINTNORMALTYPE = pcl::PointXYZINormal;
using POINTCLOUD = pcl::PointCloud<POINTTYPE>;
using PointRGB = pcl::PointXYZRGB;


#define Lidar_FOV_Width 1280
#define Lidar_FOV_Height 960
#define Lidar_Focal_Length 800


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