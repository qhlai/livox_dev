#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/octree/octree.h>
#include <pcl/io/pcd_io.h>

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::cout << "Usage: ply_to_octree input.ply output.pcd" << std::endl;
        return -1;
    }

    std::string input_file = argv[1];
    std::string output_file = argv[2];

    pcl::PolygonMesh mesh;
    pcl::io::loadPLYFile(input_file, mesh);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

    pcl::octree::OctreePointCloud<pcl::PointXYZ> octree(0.01);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();

    pcl::octree::OctreePointCloud<pcl::PointXYZ>::AlignedPointTVector voxel_centers;
    octree.getOccupiedVoxelCenters(voxel_centers);

    pcl::PointCloud<pcl::PointXYZ> output_cloud;
    output_cloud.points.resize(voxel_centers.size());
    for (size_t i = 0; i < voxel_centers.size(); ++i)
    {
        output_cloud.points[i] = voxel_centers[i];
    }

    pcl::io::savePCDFileBinary(output_file, output_cloud);

    std::cout << "PLY to Octree conversion completed. Octree saved as PCD file!" << std::endl;

    return 0;
}