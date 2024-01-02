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
// #include <dbg.h>
// #include <std/shared_ptr.h>

#define USE_RTABMAP_SUB
// #define USE_FASTLIO_SUB

// class 
using POINT_TYPE = pcl::PointXYZRGB;
using POINTCLOUD = pcl::PointCloud<POINT_TYPE>;
sensor_msgs::PointCloud2 cloud_map;
sensor_msgs::PointCloud2 cloud_voxel;
std::queue<POINTCLOUD> cloud_voxel_sum_queue;
// POINTCLOUD cloud_voxel_sum;
POINTCLOUD::Ptr cloud_voxel_sum(new POINTCLOUD);

// pcl::PointCloud<POINT_TYPE> cloud_voxel_sum;
sensor_msgs::PointCloud2 cloud_map_voxel_sum_msg;
Eigen::Isometry3d eigen_transform;
ros::Publisher occupancy_grid_pub;
bool if_trans=false;

void tf_trans(){
    // tf2_ros::Buffer tfBuffer;
    
    tf::TransformListener listener;
    double extrapolation_limit = 3.0; // 设置允许的最大误差值
    listener.setExtrapolationLimit(ros::Duration(extrapolation_limit));
    std::string target_frame = "map";  // 目标坐标系
    std::string source_frame = "camera_color_optical_frame";  // 源坐标系
    listener.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(3.0));
    tf::StampedTransform transform;
    try {
        listener.lookupTransform(target_frame, source_frame, ros::Time(0), transform);
        // tf::transformTFToEigen(transform, eigen_transform);
        // 获取平移向量 
        tf::Vector3 translation = transform.getOrigin();
        eigen_transform.translation() << translation.x(), translation.y(), translation.z();

        // 获取旋转四元数
        tf::Quaternion rotation = transform.getRotation();
        
        // Eigen::Quaterniond(rotation.w(), rotation.x(), rotation.y(), rotation.z());
        auto rot = Eigen::Quaterniond(rotation.w(), rotation.x(), rotation.y(), rotation.z()).toRotationMatrix();
        // eigen_transform.rotate(rot);
        eigen_transform.linear() =rot;
        std::cout <<"rotation: "<< rotation.w()<< rotation.x()<<rotation.y()<<rotation.z()<< std::endl
                <<rot.matrix()<< std::endl;

        std::cout << std::endl << eigen_transform.matrix() << std::endl;
        if_trans = true;
    } catch (tf::TransformException& ex) {
        ROS_ERROR("%s", ex.what());
        // 处理异常情况
        if_trans = false;
    }   
}
// void calcThresholdedNodes(const octomap::OcTree tree,
//                           unsigned int& num_thresholded,
//                           unsigned int& num_other)
// {
//   num_thresholded = 0;
//   num_other = 0;

//   for(octomap::OcTree::tree_iterator it = tree.begin_tree(), end=tree.end_tree(); it!= end; ++it){
//     if (tree.isNodeAtThreshold(*it))
//       num_thresholded++;
//     else
//       num_other++;
//   }
// }

// void octmap_outputStatistics(const octomap::OcTree tree){
//   unsigned int numThresholded, numOther;
//   calcThresholdedNodes(tree, numThresholded, numOther);
//   size_t memUsage = tree.memoryUsage();
//   unsigned long long memFullGrid = tree.memoryFullGrid();
//   size_t numLeafNodes = tree.getNumLeafNodes();

//   std::cout << "Tree size: " << tree.size() <<" nodes (" << numLeafNodes<< " leafs). " <<numThresholded <<" nodes thresholded, "<< numOther << " other\n";
//   std::cout << "Memory: " << memUsage << " byte (" << memUsage/(1024.*1024.) << " MB)" << std::endl;
//   std::cout << "Full grid: "<< memFullGrid << " byte (" << memFullGrid/(1024.*1024.) << " MB)" << std::endl;
//   double x, y, z;
//   tree.getMetricSize(x, y, z);
//   std::cout << "Size: " << x << " x " << y << " x " << z << " m^3\n";
//   std::cout << std::endl;
// }
void show_octmap(){

    pcl::octree::OctreePointCloud<POINT_TYPE> octree(0.5); // 设置
    octree.setInputCloud(cloud_voxel_sum);
    octree.addPointsFromInputCloud();

 // 使用 sizeof 运算符计算 OctreePointCloud 对象的大小
    size_t octreeSize = sizeof(octree);

    std::cout << "Size of OctreePointCloud object: " << octreeSize << " bytes" << std::endl;

    pcl::PointCloud<POINT_TYPE>::Ptr cloud(new pcl::PointCloud<POINT_TYPE>);

    // 将OctreePointCloud的点添加到PointCloud中
for (auto it = octree.leaf_begin(); it != octree.leaf_end(); ++it) {
    std::vector<int> indices;
    it.getLeafContainer().getPointIndices(indices);
    std::cout << "point: "<< indices.size() <<"1:"<< octreeSize <<  std::endl;
    for (const int& index : indices) {
        pcl::PointXYZRGB point = cloud_voxel_sum->points[index];
        cloud->push_back(point);
        
    }
}

// 可选的：如果需要，重新计算点云的宽度和高度
cloud->width = cloud->size();
cloud->height = 1;

// 可选的：如果需要，设置其他 PointCloud 属性，如 is_dense
cloud->is_dense = true;

sensor_msgs::PointCloud2 cloud_msg;

pcl::toROSMsg(*cloud, cloud_msg);
cloud_msg.header.frame_id = "map";
occupancy_grid_pub.publish(cloud_msg);
//    // 创建可视化数据并发布到 RViz
// visualization_msgs::MarkerArray markers;
// visualization_msgs::Marker marker;
// marker.header.frame_id = "your_frame_id"; // 设置坐标系
// marker.type = visualization_msgs::Marker::CUBE_LIST;
// marker.scale.x = 0.1; // 设置栅格的大小
// marker.scale.y = 0.1;
// marker.scale.z = 0.1;
// marker.color.r = 0.0; // 设置颜色
// marker.color.g = 1.0;
// marker.color.b = 0.0;
// marker.color.a = 0.5; // 设置透明度
// marker.points.clear();

// // 遍历 Octree 中的叶子节点，将其转换为可视化栅格
// std::vector<int> leaf_indices;
// std::vector<POINT_TYPE>  leaf_points;
// std::vector<pcl::octree::OctreeContainerPointIndices> leaf_containers;

//     // 创建一个 pcl::PointCloud 对象，用于存储转换后的点云数据
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);

//     // 遍历 Octree 中的叶子节点，并将数据添加到点云中
//     for (auto it = octree.begin_leafs(); it != octree.end_leafs(); ++it) {
//         if (octree.isNodeOccupied(*it)) {
//             pcl::PointXYZRGB point;
//             point.x = it.getX();
//             point.y = it.getY();
//             point.z = it.getZ();
//             point.r = 255; // 设置点的颜色
//             point.g = 0;
//             point.b = 0;
//             pointcloud->points.push_back(point);
//         }
//     }

//     pointcloud->width = pointcloud->points.size();
//     pointcloud->height = 1;
// for (const int& idx : leaf_indices)
// {
//     pcl::PointXYZRGB point;
//     point = cloud->points[idx];

//     geometry_msgs::Point p;
//     p.x = point.x;
//     p.y = point.y;
//     p.z = point.z;
//     marker.points.push_back(p);
// }

// markers.markers.push_back(marker);

// // 发布可视化数据
// octree_marker_pub.publish(markers);


}

void cloud_map_cb(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    cloud_map = *msg;
    //std::cout <<"cloud_map_cb: "<< cloud_map.header.frame_id << std::endl;//map
    ROS_INFO("Received point cloud with width: %d, height: %d", msg->width, msg->height);
}

void voxel_cb(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    cloud_voxel = *msg;
    //std::cout <<"voxel_cb: "<< cloud_voxel.header.frame_id << std::endl;//camera_color_optical_frame
    POINTCLOUD cloud;
    pcl::fromROSMsg(cloud_voxel, cloud);
    // POINTCLOUD::Ptr transformedCloud(new POINTCLOUD);
    POINTCLOUD transformedCloud;
    // Eigen::Affine3d eigen_transform_Affine3d;
    // tf::transformEigenToTF(eigen_transform, eigen_transform);
    if(if_trans){
        pcl::transformPointCloud(cloud, transformedCloud, eigen_transform.matrix());
        // Eigen::Isometry3d tran =Eigen::Isometry3d::Identity();
        // pcl::transformPointCloud(cloud, transformedCloud, tran.matrix());
    }
// transformedCloud=cloud;
    cloud_voxel_sum_queue.push(transformedCloud);
    if(cloud_voxel_sum_queue.size() > 10) {
        cloud_voxel_sum_queue.pop();
    }
    ROS_INFO("Received point cloud with width: %d, height: %d", msg->width, msg->height);
}

bool gen_sum_pc(){
    if(cloud_voxel_sum_queue.size() == 0) {
        ROS_INFO("cloud_voxel_sum is empty");
        return false;
    }
    // ROS_INFO("save_cb");
    pcl::fromROSMsg(cloud_map, *cloud_voxel_sum);
    // cloud += cloud_map;
    for(uint32_t i = 0; i < cloud_voxel_sum_queue.size(); i++) {
        *cloud_voxel_sum += cloud_voxel_sum_queue.front();
        // cloud_voxel_sum.pop();
    }


#if 1
static uint16_t voxel_cout = 0;
if(voxel_cout >= 30)
{
    voxel_cout=0;
#if 1
{
    pcl::PointCloud<POINT_TYPE>::Ptr cloud_ptr(new pcl::PointCloud<POINT_TYPE>(*cloud_voxel_sum));
    pcl::PointCloud<POINT_TYPE>::Ptr cloud_filtered(new pcl::PointCloud<POINT_TYPE>);
    pcl::StatisticalOutlierRemoval<POINT_TYPE> sor;
    sor.setInputCloud(cloud_ptr);
    sor.setMeanK(30);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);
    *cloud_voxel_sum = *cloud_filtered;
}
#endif

#if 1
    {
        // voxel_cout=0;
        pcl::PointCloud<POINT_TYPE>::Ptr cloud_ptr(new pcl::PointCloud<POINT_TYPE>(*cloud_voxel_sum));
        pcl::PointCloud<POINT_TYPE>::Ptr cloud_filtered(new pcl::PointCloud<POINT_TYPE>);
        pcl::VoxelGrid<POINT_TYPE> sor;
        sor.setInputCloud(cloud_ptr);
        sor.setLeafSize(0.01, 0.01, 0.01);
        sor.filter(*cloud_filtered);
        *cloud_voxel_sum = *cloud_filtered;
    }
#endif

}
#endif

    std::cout <<"storage analyse: "<< cloud_voxel_sum->size() << ", " << cloud_voxel_sum_queue.size() << std::endl;
    return true;
    // pcl::toROSMsg(cloud_voxel_sum, cloud_map_voxel_sum_msg);
}
bool save_pc_cb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
#if 0
    if(cloud_map.width*cloud_map.height == 0) {
        ROS_INFO("cloud_map is empty");
        return false;
    }
    ROS_INFO("save_cb");
    pcl::PointCloud<POINT_TYPE> cloud;
    pcl::fromROSMsg(cloud_map, cloud);
    pcl::io::savePCDFileASCII("test_pcd.pcd", cloud);
    return true;
#endif
#if 0
    if(cloud_voxel_sum_queue.size() == 0) {
        ROS_INFO("cloud_voxel_sum is empty");
        return false;
    }
    ROS_INFO("save_cb");
    pcl::PointCloud<POINT_TYPE> cloud;
    pcl::fromROSMsg(cloud_map, cloud);
    // cloud += cloud_map;
    for(uint32_t i = 0; i < cloud_voxel_sum_queue.size(); i++) {
        cloud += cloud_voxel_sum_queue.front();
        // cloud_voxel_sum.pop();
    }
    pcl::io::savePCDFileASCII("test_pcd.pcd", cloud);
    return true;
#endif

#if 1
    if(gen_sum_pc()){
        pcl::io::savePCDFileASCII("~/output/test_pcd.pcd", *cloud_voxel_sum);
        return true;
    }
    else{
        return false;
    }
    
#endif
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_subscriber");
    ros::NodeHandle nh;

    ros::Publisher  pub_cloudmap = nh.advertise<sensor_msgs::PointCloud2>("/cloud_map_sum", 1);
    ros::ServiceServer service_savemap = nh.advertiseService("save_pc", save_pc_cb);
    // ros::Publisher occupancy_grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("occupancy_grid", 1);
    occupancy_grid_pub = nh.advertise<sensor_msgs::PointCloud2>("octree_markers", 1);

    ros::Subscriber sub_odometry = nh.subscribe<nav_msgs::Odometry>("/rtabmap/odom", 1, cloud_map_cb);
#ifdef USE_RTABMAP_SUB
    tf_trans();

    ros::Subscriber sub_cloudmap = nh.subscribe<sensor_msgs::PointCloud2>("/rtabmap/cloud_map", 1, cloud_map_cb);
    ros::Subscriber sub_voxel = nh.subscribe<sensor_msgs::PointCloud2>("/voxel_cloud", 1, voxel_cb);
    
    // ros::Subscriber sub_voxel = nh.subscribe<sensor_msgs::PointCloud2>("/voxel", 1, voxel_cb);
    // ros::ServiceServer service_savemap = nh.advertiseService("save_pc", save_pc_cb);
    // ros::Publisher map_pub = nh.advertise<occupancy_map_msgs::OccupancyMap>("/octomap_binary", 1);
#elif USE_FASTLIO_SUB
    std::cout << "pcd size:"<< cloud_voxel_sum.size() << std::endl;
#else 
    // std::string input_file = argv[1];
    std::string input_file = "/home/uestc/dataset/test_pcd.pcd";
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(input_file, *cloud_voxel_sum) == -1)
    {
        std::cout << "Failed to load PCD file: " << input_file << std::endl;
        return -1;
    }
    std::cout << "pcd size:"<< cloud_voxel_sum->size() << std::endl;

#endif

    ros::Rate rate(3);
    while (ros::ok())
    {
#ifdef USE_RTABMAP_SUB
        gen_sum_pc();
        pcl::toROSMsg(*cloud_voxel_sum, cloud_map_voxel_sum_msg);
        pub_cloudmap.publish(cloud_map_voxel_sum_msg);
#elif USE_FASTLIO_SUB
        std::cout << "pcd size:"<< cloud_voxel_sum.size() << std::endl;
#else
        pcl::toROSMsg(*cloud_voxel_sum, cloud_map_voxel_sum_msg);
        cloud_map_voxel_sum_msg.header.frame_id = "map";
        pub_cloudmap.publish(cloud_map_voxel_sum_msg);
        show_octmap();
#endif


        ros::spinOnce();
        ros::Duration(rate).sleep();
    }
    
    ros::spin();

    return 0;
}
