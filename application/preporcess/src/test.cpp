
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include "tools/tools_utils.hpp"
// #include "tools/tools_value_redefine.hpp"
// #include "tools/tools_logger_lite.hpp"

using POINTCLOUD = pcl::PointXYZRGB;

void pointcloud_size(pcl::PointCloud<POINTCLOUD>::Ptr cloud){
    // size_t cloudSize = sizeof(*cloud);
    // size_t cloudSize = pcl::getPointCloudSize(*cloud);
    // size_t numPoints = *cloud.size();
    size_t numPoints = cloud->width * cloud->height;
    size_t pointSize = sizeof(cloud->points[0]);
    size_t totalSize = numPoints * pointSize;
    std::cout << "PointCloud 的内存占用为: " << totalSize << " 字节" << std::endl; 
}

loglevel_e loglevel = logERROR;

int main(int argc, char** argv)
{
    log_test();
    dbg(42, "hello world", false);
    
    return 0;
}