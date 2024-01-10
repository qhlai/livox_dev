#pragma once
#include <iostream>

 #include <pcl/io/pcd_io.h>


using POINTTYPE = pcl::PointXYZ;
using POINTNORMALTYPE = pcl::PointXYZINormal;
using POINTCLOUD = pcl::PointCloud<POINTTYPE>;
using PointRGB = pcl::PointXYZRGB;

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