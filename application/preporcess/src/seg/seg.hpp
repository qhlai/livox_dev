#pragma once

// #include <pcl/io/pcd_io.h>
#include "../base/common.hpp"
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
namespace PointCloud_process{



#define DisThre 0.03//平面分割阈值

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
    void output_plane(pcl::PointCloud<PointRGB>::Ptr cloud_plane,int begin);
    void cloudPassThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,const char *axis,int min,int max);
};




};

