
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
// #include <dbg-macro/dbg.h>

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

    uint8_t numChars = 4; // 要获取的末尾字符数,即pcd 或者 ply

    std::string pc_type;
    if (numChars <= input_file.length()) {
        pc_type = input_file.substr(input_file.length() - numChars);
        // std::cout << "末尾 " << numChars << " 个字符为: " << pc_type << std::endl;
    } else {
        std::cout << "字符串长度不足 " << numChars << " 个字符。" << std::endl;
    }
#else
    std::string input_file = argv[1];
    std::string output_file = argv[2];
#endif

    pcl::PointCloud<POINTCLOUD>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    if(pc_type == ".pcd"){
        if (pcl::io::loadPCDFile<POINTCLOUD>(input_file, *cloud) == -1)
        {
            std::cout << "Failed to load PCD file: " << input_file << std::endl;
            return -1;
        }
    }
    else if(pc_type == ".ply"){

        pcl::PolygonMesh mesh;
        if (pcl::io::loadPLYFile(input_file, mesh) == -1)
        {
            std::cout << "Failed to load PLY file: " << input_file << std::endl;
            return -1;
        } 
        pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
    }
    else{
        std::cout << "Failed to load PCD file: " << input_file << std::endl;
    }
    pointcloud_size(cloud);
    // size_t cloudSize = sizeof(*cloud);
    // std::cout << "PointCloud 的内存占用为: " << cloudSize << " 字节" << std::endl;


    pcl::PLYWriter ply_writer;
    ply_writer.write(output_file, *cloud);

    std::cout << "PCD to PLY conversion completed!" << std::endl;

    return 0;
}