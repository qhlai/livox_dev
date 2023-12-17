#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/octree/octree.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: visualize_octree input.ply" << std::endl;
        return -1;
    }

    std::string input_file = argv[1];

    pcl::PolygonMesh mesh;
    pcl::io::loadPLYFile(input_file, mesh);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

    pcl::octree::OctreePointCloud<pcl::PointXYZ> octree(0.01);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();

    pcl::visualization::PCLVisualizer viewer("Octree Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    viewer.addCoordinateSystem(1.0);

    pcl::octree::OctreePointCloud<pcl::PointXYZ>::LeafNodeIterator it;
    pcl::octree::OctreePointCloud<pcl::PointXYZ>::LeafNodeIterator it_end = octree.leaf_end();
    for (it = octree.leaf_begin(); it != it_end; ++it)
    {
        pcl::octree::OctreePointCloud<pcl::PointXYZ>::LeafContainer leaf = *it;
        Eigen::Vector3f voxel_min, voxel_max;
        octree.getVoxelBounds(leaf, voxel_min, voxel_max);
        viewer.addCube(voxel_min[0], voxel_max[0], voxel_min[1], voxel_max[1], voxel_min[2], voxel_max[2], 1.0, 1.0, 1.0);
    }

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }

    return 0;
}