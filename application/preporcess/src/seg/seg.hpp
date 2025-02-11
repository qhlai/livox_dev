#pragma once

// #include <pcl/io/pcd_io.h>
#include "../base/common.hpp"
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>
namespace PointCloud_process{

// extern loglevel_e loglevel;

#define DisThre 0.05//平面分割阈值

// #define Lidar_FOV_Width 1280
// #define Lidar_FOV_Height 960
// #define Lidar_Focal_Length 800
// #define Lidar_FOV_Width 640
// #define Lidar_FOV_Height 480
// #define Lidar_Focal_Length 616


template <typename PointTy>
void pointcloud_size(typename pcl::PointCloud<PointTy>::Ptr cloud) {
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
    // pcl::search::KdTree<POINTTYPE>::Ptr tree;
    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("planar segment"));
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("planar segment"));
    // POINTCLOUD::Ptr cloud(new POINTCLOUD);
    // POINTCLOUD::Ptr cloud_inner(new POINTCLOUD);
    // POINTCLOUD::Ptr cloud_outer(new POINTCLOUD);
    virtual auto clac_normal(pcl::PointCloud<POINTTYPE>::Ptr cloud_input)->pcl::PointCloud<pcl::Normal>::Ptr;

    virtual auto Plane_fitting(pcl::PointCloud<POINTTYPE>::Ptr cloud_input)->void;
    virtual auto Plane_fitting_normal(pcl::PointCloud<POINTTYPE>::Ptr cloud_input)->void;
    virtual auto Plane_fitting_cluster_eu(pcl::PointCloud<POINTTYPE>::Ptr cloud_input)->void;
    virtual auto Plane_fitting_cluster_growth(pcl::PointCloud<POINTTYPE>::Ptr cloud_input)->void;
    virtual auto Plane_fitting_cluster_growth_v(pcl::PointCloud<POINTTYPE>::Ptr cloud_input)->void;
    // template <typename PointTy>
    // cv::Mat projection(typename pcl::PointCloud<PointTy>::Ptr cloud);
    virtual auto projection(POINTCLOUD::Ptr cloud)->cv::Mat ;
    // cv::Mat projection(pcl::PointCloud<PointTy>::Ptr cloud);
    virtual auto normal_viz(pcl::PointCloud<POINTTYPE>::Ptr cloud_input)->void;
    
    virtual auto backprojection(POINTCLOUD::Ptr cloud)->void;
    virtual auto output_plane(pcl::PointCloud<PointRGB>::Ptr cloud_plane,int begin)->void;
    virtual auto cloudPassThrough(pcl::PointCloud<POINTTYPE>::Ptr cloud,const char *axis,int min,int max)->void;

public:
virtual auto addSupervoxelConnectionsToViewer (POINTTYPE &supervoxel_center,
                                       POINTCLOUD &adjacent_supervoxel_centers,
                                       std::string supervoxel_name,
                                       pcl::visualization::PCLVisualizer::Ptr & viewer)->void;
virtual auto get_cmd_parm()->void;

public:


    bool disable_transform = false;
    bool voxel_res_specified = false;
    float voxel_resolution = 0.008f;
    float seed_resolution = 0.1f;
    float color_importance = 0.2f;
    float spatial_importance = 0.4f;
    float normal_importance = 1.0f;
    float normal_threshold = 0.8f;
    float curvature_threshold = 0.8f;
    // float min_cluster_size = 100;
    // float max_cluster_size = 1000000;
    // float cluster_tolerance = 0.1;
    // float cluster_tolerance_eu = 0.1;
    // float cluster_tolerance_growth = 0.1;
    // float cluster_tolerance_growth_v = 0.1;
    // float cluster_tolerance_normal = 0.1;
    // float cluster_tolerance_normal_v = 0.1;
    // float cluster_tolerance_normal_eu = 0.1;
    // float cluster_tolerance_normal_eu = 0.1;

};

// #include "seg.tpp" 


};

