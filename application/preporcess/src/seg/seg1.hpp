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

// pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_merge(pcl::PointCloud<pcl::PointXYZI>::Ptr& xyz_cloud, pcl::PointCloud<pcl::Normal>::Ptr& normals_cloud){
//     pcl::PointCloud<pcl::PointXYZINormal>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
//     // 设置 combined_cloud 的大小
//     combined_cloud->resize(normals_cloud->size());

//     // 将 normals_cloud 中的法线数据复制到 combined_cloud 中
//     for (size_t i = 0; i < normals_cloud->size(); ++i) {
//         combined_cloud->points[i].x = xyz_cloud->points[i].x;
//         combined_cloud->points[i].y = xyz_cloud->points[i].y;
//         combined_cloud->points[i].z = xyz_cloud->points[i].z;
//         combined_cloud->points[i].intensity = xyz_cloud->points[i].intensity;
//         combined_cloud->points[i].normal_x = normals_cloud->points[i].normal_x;
//         combined_cloud->points[i].normal_y = normals_cloud->points[i].normal_y;
//         combined_cloud->points[i].normal_z = normals_cloud->points[i].normal_z;
//     }
//     return combined_cloud;

// }

template <typename PointT>
class Segment{
public:

    using PointCloudT = pcl::PointCloud<PointT>;
    using NormalCloudT = pcl::PointCloud<pcl::Normal>;
    using SearchKdtreeT = pcl::search::KdTree<PointT>;
    // using PointSVT = pcl::PointXYZRGBA;
    using PointLT = pcl::PointXYZL;
    // using PointCloudSVT = pcl::PointCloud<PointSVT>;
    using PointNT =pcl::PointNormal;
    using  PointNCloudT =pcl::PointCloud<PointNT>;
    // using PointT = PointT;
    // typedef pcl::PointNormal PointNT;

    // typedef pcl::PointCloud<PointNT> PointNCloudT;
    // typename PointCloudSVT::Ptr cloud_sv;
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
    virtual auto load_pointcloud(std::string path)->void;
    virtual auto save_pointcloud(std::string path)->void;
    virtual auto pointcloud_finetune()->void;
    // virtual auto Plane_fitting(typename PointCloudT::Ptr cloud_input)->void;
    virtual auto Plane_fitting_normal(typename PointCloudT::Ptr cloud_input)->void;
    virtual auto Plane_fitting_cluster_eu(typename PointCloudT::Ptr cloud_input)->void;
    virtual auto Plane_fitting_cluster_growth(typename PointCloudT::Ptr cloud_input)->void;
    virtual auto Plane_fitting_cluster_growth_v(typename PointCloudT::Ptr cloud_input)->void;

    virtual auto Cluster_super_voxel(typename PointCloudT::Ptr cloud_input)->void;

    virtual auto addSupervoxelConnectionsToViewer (PointT &supervoxel_center,
                                  PointCloudT &adjacent_supervoxel_centers,
                                  std::string supervoxel_name,
                                  pcl::visualization::PCLVisualizer::Ptr & viewer)->void;

    // // template <typename PointTy>
    // cv::Mat projection(typename pcl::PointCloud<PointTy>::Ptr cloud);
    // virtual auto projection(POINTCLOUD::Ptr cloud)->cv::Mat ;
    //cv::Mat projection(pcl::PointCloud<PointTy>::Ptr cloud);
    virtual auto normal_viz(typename PointCloudT::Ptr cloud_input)->void;
    
    // virtual auto backprojection(POINTCLOUD::Ptr cloud)->void;
    virtual auto output_plane(pcl::PointCloud<PointRGB>::Ptr cloud_plane,int begin)->void;
    virtual auto output_plane(pcl::PolygonMesh::Ptr cloud_plane,int begin)->void;
    virtual auto point2mesh(typename PointCloudT::Ptr cloud_input,pcl::Normal normal)->pcl::PolygonMesh::Ptr;
    virtual auto cloudPassThrough(typename PointCloudT::Ptr cloud,const char *axis,int min,int max)->void;
    virtual auto SmoothPointcloud()->typename PointCloudT::Ptr;
    virtual auto greedy_traingle_GenerateMesh(typename PointCloudT::Ptr cloud_in,pcl::PointCloud<pcl::Normal>::Ptr normals)->pcl::PolygonMesh::Ptr;

    virtual auto poisson_reconstruction_GenerateMesh(typename PointCloudT::Ptr cloud_in,pcl::PointCloud<pcl::Normal>::Ptr normals)->pcl::PolygonMesh::Ptr;

    virtual auto get_cmd_parm(int argc, char** argv)->bool;

    virtual auto init_display()->void;
    virtual auto display()->void;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    
private:
    uint8_t R;
    uint8_t G;
    uint8_t B; 

    // virtual auto get_cmd_parm()->void;
public:

    int display_type = 0;
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

