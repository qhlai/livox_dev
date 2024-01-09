
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
// #include "tools/tools_utils.hpp"


#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "tools/tools_value_redefine.hpp"
#include "tools/tools_logger_lite.hpp"

using POINTTYPE = pcl::PointXYZI;
using POINTCLOUD = pcl::PointCloud<POINTTYPE>;

void pointcloud_size(POINTCLOUD::Ptr cloud){
    // size_t cloudSize = sizeof(*cloud);
    // size_t cloudSize = pcl::getPointCloudSize(*cloud);
    // size_t numPoints = *cloud.size();
    size_t numPoints = cloud->width * cloud->height;
    size_t pointSize = sizeof(cloud->points[0]);
    size_t totalSize = numPoints * pointSize;
    std::cout << "PointCloud 的内存占用为: " << totalSize << " 字节" << std::endl; 
}

loglevel_e loglevel = logDEBUG;

int main(int argc, char** argv)
{
    // log_test();
    // dbg(42, "hello world", false);
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::string input_file;
    std::string output_file;
    if (argc < 3)
    {
        std::cout << "Usage: pcd_to_ply input.pcd output.ply" << std::endl;

        logit(logWARN) << "defalut " ;
        input_file ="/home/uestc/ros/mid360_ws/src/application/preporcess/dataset/2.pcd";
        output_file ="/home/uestc/ros/mid360_ws/src/application/preporcess/dataset/3.pcd";
        // dbg(input_file, output_file);
        // return -1;
    }else{
        input_file = argv[1];
        output_file = argv[2];
    }

    POINTCLOUD::Ptr cloud(new POINTCLOUD);
    if (pcl::io::loadPCDFile<POINTTYPE>(input_file, *cloud) == -1)
    {
        // log(logFATAL) << "Failed to load PCD file: " << input_file ;
        return -1;

    }
    // 创建法向量估计对象
    pcl::NormalEstimation<POINTTYPE, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    // 创建一个用于存储法向量的PointCloud
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    // 估计法向量
    pcl::search::KdTree<POINTTYPE>::Ptr tree(new pcl::search::KdTree<POINTTYPE>);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.03);  // 设置法向量估计半径
    ne.compute(*cloud_normals);

    // 创建分割对象
    pcl::SACSegmentationFromNormals<POINTTYPE, pcl::Normal> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.02);  // 设置距离阈值


    // 设置输入点云和法向量
    seg.setInputCloud(cloud);
    seg.setInputNormals(cloud_normals);

    // 创建一个对象来存储分割结果
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    // 分割平面
    seg.segment(*inliers, *coefficients);

    // 使用分割结果提取平面点云
    POINTCLOUD::Ptr cloud_plane(new POINTCLOUD);
    pcl::ExtractIndices<POINTTYPE> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_plane);

    pcl::PCDWriter writer;
    writer.write(output_file, *cloud_plane);
    
    return 0;
}