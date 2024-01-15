#pragma once

// #include <pcl/io/pcd_io.h>
#include "../base/common.hpp"
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>
namespace PointCloud_process{

// extern loglevel_e loglevel;

#define DisThre 0.03//平面分割阈值

// #define Lidar_FOV_Width 1280
// #define Lidar_FOV_Height 960
// #define Lidar_Focal_Length 800
// #define Lidar_FOV_Width 640
// #define Lidar_FOV_Height 480
// #define Lidar_Focal_Length 616


template <typename PointT>
void pointcloud_size(typename pcl::PointCloud<PointT>::Ptr cloud) {
    size_t numPoints = cloud->width * cloud->height;
    size_t pointSize = sizeof(cloud->points[0]);
    size_t totalSize = numPoints * pointSize;
    std::cout << "PointCloud 的内存占用为: " << totalSize << " 字节. " <<cloud->width <<","<< cloud->height <<","<<sizeof(cloud->points[0])<< std::endl;
}

class Segment{
public:
    Segment();
    uint8_t R;
    uint8_t G;
    uint8_t B; 

    pcl::PointCloud<PointRGB>::Ptr cloud_all;

    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("planar segment"));
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("planar segment"));
    // POINTCLOUD::Ptr cloud(new POINTCLOUD);
    // POINTCLOUD::Ptr cloud_inner(new POINTCLOUD);
    // POINTCLOUD::Ptr cloud_outer(new POINTCLOUD);

    void Plane_fitting(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input);
    void Plane_fitting_normal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input);
    // template <typename PointT>
    // cv::Mat projection(typename pcl::PointCloud<PointT>::Ptr cloud);
    cv::Mat projection(POINTCLOUD::Ptr cloud);
    // cv::Mat projection(pcl::PointCloud<PointT>::Ptr cloud);

    void backprojection(POINTCLOUD::Ptr cloud);
    void output_plane(pcl::PointCloud<PointRGB>::Ptr cloud_plane,int begin);
    void cloudPassThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,const char *axis,int min,int max);
};

// #include "seg.tpp" 


};

