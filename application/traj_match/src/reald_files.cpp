#include <iostream>
#include <limits>
#include <iostream>
#include <limits>
#include <exception>
#include <queue>

#include <ros/ros.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <std_srvs/Empty.h>
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/conversions.h>   
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
// #include <pcl/octomap/octomap.h>


#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
// #include <occupancy_map_msgs/OccupancyMap.h>
#include <visualization_msgs/MarkerArray.h>

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>
#include <fstream>
#include <string>
// #include <dbg.h>
// #include <std/shared_ptr.h>



int main() {
    std::ifstream file("example.txt"); // 打开文件

    if (!file.is_open()) {
        std::cerr << "无法打开文件" << std::endl;
        return 1;
    }

    std::string line;
    std::vector<std::vector<float>> pose_v;
    int line_count = 0;

    while (std::getline(file, line)) { // 逐行读取文件
        line_count++;

        if (line_count % 5 == 0) { // 每五行进行处理
            std::istringstream iss(line);
            std::string item;
            std::vector<float> pose_items;

            while (std::getline(iss, item, ',')) { // 拆分每一行的内容
                pose_items.push_back(std::stof(item)); // 转换为浮点数并保存
            }

            if (pose_items.size() == 7) { // 确保有足够的项目
                pose_v.push_back(pose_items); // 将位姿添加到容器中
            }
        }
    }

    file.close(); // 关闭文件

    // 现在，pose_v 中包含每五行的位姿数据
    for (const auto& pose : pose_v) {
        for (float item : pose) {
            std::cout << item << " ";
        }
        std::cout << std::endl;
    }

    return 0;
}