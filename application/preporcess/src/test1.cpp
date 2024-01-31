
#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
// #include "tools/tools_utils.hpp"


#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>  //直通滤波器头文件al_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include<pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>  //直通滤波器头文件

#include "base/common.hpp"
#include "seg/seg1.hpp"
// #include "seg/seg1.hpp"


#include "tools/tools_value_redefine.hpp"
// #include "tools/tools_logger_lite.hpp"

#include "map/rbg_pointcloud.hpp"
#include <boost/thread/thread.hpp>


// using POINTTYPE = pcl::PointXYZ;
// using POINTNORMALTYPE = pcl::PointXYZINormal;
// using POINTCLOUD = pcl::PointCloud<POINTTYPE>;
// using PointRGB = pcl::PointXYZRGB;

// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("planar segment"));

// https://blog.csdn.net/xx970829/article/details/123751443



FILE *fpWrite=fopen("cloud_all.txt","w");//a续写，w清除后写入



pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_merge(pcl::PointCloud<pcl::PointXYZI>::Ptr& xyz_cloud, pcl::PointCloud<pcl::Normal>::Ptr& normals_cloud){
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    // 设置 combined_cloud 的大小
    combined_cloud->resize(normals_cloud->size());

    // 将 normals_cloud 中的法线数据复制到 combined_cloud 中
    for (size_t i = 0; i < normals_cloud->size(); ++i) {
        combined_cloud->points[i].x = xyz_cloud->points[i].x;
        combined_cloud->points[i].y = xyz_cloud->points[i].y;
        combined_cloud->points[i].z = xyz_cloud->points[i].z;
        combined_cloud->points[i].intensity = xyz_cloud->points[i].intensity;
        combined_cloud->points[i].normal_x = normals_cloud->points[i].normal_x;
        combined_cloud->points[i].normal_y = normals_cloud->points[i].normal_y;
        combined_cloud->points[i].normal_z = normals_cloud->points[i].normal_z;
    }
    return combined_cloud;

}

// loglevel_e loglevel = logDEBUG4;

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

        // logit(logWARN) << "defalut " ;
        std::string path_prefix="/home/uestc/ros/mid360_ws/src/application/preporcess/dataset/";

        input_file =path_prefix+"2.pcd";
        output_file =path_prefix+"3.pcd";
        // dbg(input_file, output_file);
        // return -1;
    }else{
        input_file = argv[1];
        output_file = argv[2];
    }

    PointCloud_process1::Segment<pcl::PointXYZI> seg;

    // if(!seg.get_cmd_parm(argc, argv)){
    //     return 1;
    // }
    
    // PointCloud_process::Segment seg;
    POINTCLOUD::Ptr cloud(new POINTCLOUD);
    POINTCLOUD::Ptr cloud_inner(new POINTCLOUD);
    POINTCLOUD::Ptr cloud_outer(new POINTCLOUD);

    if (pcl::io::loadPCDFile<POINTTYPE>(input_file, *cloud) == -1)
    {
        // log(logFATAL) << "Failed to load PCD file: " << input_file ;
        // logit(logFATAL) << "Failed to load PCD file " ;
        return -1;

    }
    // pcl::io::loadPointCloud()
    seg.viewer->setWindowName("Plane Model Segmentation");
    seg.viewer->setBackgroundColor(0, 0, 0);
    //直通滤波
    seg.cloudPassThrough(cloud,"y",-20,20);
    seg.cloudPassThrough(cloud,"x",5,50);
    seg.cloudPassThrough(cloud,"z",-5,15);

    // seg.projection(cloud);

    // seg.Plane_fitting(cloud);
    
    // seg.normal_viz(cloud);
    seg.Plane_fitting_cluster_growth(cloud);
    for(std::size_t i=0; i< seg.cloud_all->size(); i++)
        fprintf(fpWrite,"%2.3f %2.3f %2.3f %d %d %d \n",seg.cloud_all->points[i].x,seg.cloud_all->points[i].y,seg.cloud_all->points[i].z,seg.cloud_all->points[i].r,seg.cloud_all->points[i].g,seg.cloud_all->points[i].b);
    fclose(fpWrite);
    cout << "cloud_all.txt 保存完毕！！！" << endl;

    while (!seg.viewer->wasStopped())
    {
        seg.viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

	return (0);

}