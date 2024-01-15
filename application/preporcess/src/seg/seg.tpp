#pragma once

// #include <pcl/io/pcd_io.h>
#include "../base/common.hpp"
#include <boost/thread/thread.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>  //直通滤波器头文件
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>
namespace PointCloud_process{

template <typename PointT>
cv::Mat Segment::projection(typename pcl::PointCloud<PointT>::Ptr cloud){

    //pcl_viewer '/home/hanglok/pc_ws/src/application/preporcess/dataset/1.pcd'  -ax 5

    cv::Mat depthMap = cv::Mat::zeros(Lidar_FOV_Height, Lidar_FOV_Width, CV_32FC1);
        // 遍历点云，更新深度图

#define Lidar_Axis_Trans 0
#if Lidar_Axis_Trans
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform(0,0)=0;transform(0,1)=-1;transform(0,2)=0;transform(0,3)=0;
    transform(1,0)=1;transform(1,1)=0;transform(1,2)=0;transform(1,3)=0;
    transform(2,0)=0;transform(2,1)=0;transform(2,2)=1;transform(2,3)=0;
    transform(3,0)=0;transform(3,1)=0;transform(3,2)=0;transform(3,3)=1;
    // 应用变换
    POINTCLOUD::Ptr transformed_cloud(new POINTCLOUD());
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);

    for (const auto& point : transformed_cloud->points) {
        
#else
    for (const auto& point : cloud->points) {
#endif

#if Lidar_Axis_Trans
        // 假设点云已经在相机坐标系中
        int u = static_cast<int>(point.x * Lidar_Focal_Length / point.z + Lidar_FOV_Width / 2);
        int v = static_cast<int>(point.y * Lidar_Focal_Length / point.z + Lidar_FOV_Height / 2);
#else
        // 机器人坐标系
        int u = static_cast<int>(-point.y * Lidar_Focal_Length / point.x + Lidar_FOV_Width / 2);
        int v = static_cast<int>(-point.z * Lidar_Focal_Length / point.x + Lidar_FOV_Height / 2);
#endif
        
        if (u >= 0 && u < Lidar_FOV_Width && v >= 0 && v < Lidar_FOV_Height) {
            depthMap.at<float>(v, u) = point.x;
            std::cout << u << ", "<< v << std::endl;
        }
    }
    // 转换深度图为可视化格式（可选）
    cv::Mat displayMap;
    cv::normalize(depthMap, displayMap, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::imwrite("/home/uestc/pc_ws/src/1.png",displayMap);
    cv::imshow("Depth Map", displayMap);
    cv::waitKey(0);
    return depthMap;

}


};

