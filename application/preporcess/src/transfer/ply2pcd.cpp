
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

int main(int argc, char** argv)
{
    #if 1
    if (argc < 3)
    {
        std::cout << "Usage: pcd_to_ply input.pcd output.ply" << std::endl;
        return -1;
    }

    std::string input_file = argv[1];
    std::string output_file = argv[2];
#else
    std::string input_file = argv[1];
    std::string output_file = argv[2];
#endif
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(input_file, *cloud) == -1)
    {
        std::cout << "Failed to load PCD file: " << input_file << std::endl;
        return -1;
    }

    pcl::PLYWriter ply_writer;
    ply_writer.write(output_file, *cloud);

    std::cout << "PCD to PLY conversion completed!" << std::endl;

    return 0;
}