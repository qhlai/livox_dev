#pragma once

// #include <pcl/io/pcd_io.h>
#include "../base/common.hpp"
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>


#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>//使用OMP需要添加的头文件
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>  //直通滤波器头文件
#include <pcl/common/transforms.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vtkPolyLine.h>

namespace PointCloud_process1{

#define DisThre 0.05//平面分割阈值
// extern loglevel_e loglevel;

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


template <typename PointT>
class Segment{
public:

    using PointCloudT = pcl::PointCloud<PointT>;
    using NormalCloudT = pcl::PointCloud<pcl::Normal>;
    using SearchKdtreeT = pcl::search::KdTree<PointT>;

    typename PointCloudT::Ptr cloud;
    typename PointCloudT::Ptr cloud_inner;
    typename PointCloudT::Ptr cloud_outer;
    pcl::PointCloud<PointRGB>::Ptr cloud_all;
public:
    Segment(){
        cloud = typename PointCloudT::Ptr(new PointCloudT);
        cloud_inner = typename PointCloudT::Ptr(new PointCloudT);
        cloud_outer = typename PointCloudT::Ptr(new PointCloudT);
        cloud_all = pcl::PointCloud<PointRGB>::Ptr(new pcl::PointCloud<PointRGB>);
        viewer=boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("planar segment"));

    }

    // virtual auto pointcloud_size(typename PointCloudT::Ptr cloud)->void{
    //     size_t numPoints = cloud->width * cloud->height;
    //     size_t pointSize = sizeof(cloud->points[0]);
    //     size_t totalSize = numPoints * pointSize;
    //     std::cout << "PointCloud 的内存占用为: " << totalSize << " 字节. " <<cloud->width <<","<< cloud->height <<","<<sizeof(cloud->points[0])<< std::endl;
    // }   

    // template <typename PointT>
    virtual auto clac_normal(typename PointCloudT::Ptr cloud_input)->pcl::PointCloud<pcl::Normal>::Ptr;
    
    // virtual auto Plane_fitting(typename PointCloudT::Ptr cloud_input)->void;
    virtual auto Plane_fitting_normal(typename PointCloudT::Ptr cloud_input)->void;
    virtual auto Plane_fitting_cluster_eu(typename PointCloudT::Ptr cloud_input)->void;
    virtual auto Plane_fitting_cluster_growth(typename PointCloudT::Ptr cloud_input)->void;
    virtual auto Plane_fitting_cluster_growth_v(typename PointCloudT::Ptr cloud_input)->void;
    // // template <typename PointTy>
    // cv::Mat projection(typename pcl::PointCloud<PointTy>::Ptr cloud);
    // virtual auto projection(POINTCLOUD::Ptr cloud)->cv::Mat ;
    //cv::Mat projection(pcl::PointCloud<PointTy>::Ptr cloud);
    virtual auto normal_viz(typename PointCloudT::Ptr cloud_input)->void;
    
    // virtual auto backprojection(POINTCLOUD::Ptr cloud)->void;
    virtual auto output_plane(pcl::PointCloud<PointRGB>::Ptr cloud_plane,int begin)->void;
    virtual auto cloudPassThrough(typename PointCloudT::Ptr cloud,const char *axis,int min,int max)->void;

    virtual auto get_cmd_parm(int argc, char** argv)->bool;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
private:
    uint8_t R;
    uint8_t G;
    uint8_t B; 

    // virtual auto get_cmd_parm()->void;
public:


    bool disable_transform = false;
    bool voxel_res_specified = false;
    u32 normal_min = 100;
    
    float voxel_resolution = 0.008f;
    float seed_resolution = 0.1f;
    float color_importance = 0.2f;
    float spatial_importance = 0.4f;
    float normal_importance = 1.0f;
    float normal_threshold = 0.8f;
    float curvature_threshold = 0.8f;

};



};

