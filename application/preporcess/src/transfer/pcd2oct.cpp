#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree.h>

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: pcd_to_octree input.pcd" << std::endl;
        return -1;
    }

    std::string input_file = argv[1];

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_file, *cloud) == -1)
    {
        std::cout << "Failed to load PCD file: " << input_file << std::endl;
        return -1;
    }

    pcl::octree::OctreePointCloud<pcl::PointXYZ> octree(0.01);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();

    // 八叉树构建完成后，可以进行其他操作，例如查询八叉树节点、提取体素等

    std::cout << "PCD to Octree conversion completed!" << std::endl;

    return 0;
}