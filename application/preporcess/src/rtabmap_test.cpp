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

#include <queue>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
// #include <occupancy_map_msgs/OccupancyMap.h>
#include <visualization_msgs/MarkerArray.h>

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
// #include <std/shared_ptr.h>

// #define USE_RTABMAP_SUB
// #define USE_FASTLIO_SUB

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

void show_octmap(){

    pcl::octree::OctreePointCloud<POINT_TYPE> octree(0.01); // 设置
    octree.setInputCloud(cloud_voxel_sum);
    octree.addPointsFromInputCloud();

    // 执行八叉树构建
    // octree.buildOctree();
// 获取 Octree 的边界框
Eigen::Vector3f min_pt, max_pt;
// octree.getBoundingBox(min_pt, max_pt);
 // 获取 Octree 中所有点的边界框
// Eigen::Vector4f min_pt, max_pt;
// pcl::getMinMax3D(*cloud_voxel_sum, min_pt, max_pt);
min_pt<< -10 ,-10 ,-10;
max_pt<< 10 ,10 ,10;
float x_range = max_pt.x() - min_pt.x();
float y_range = max_pt.y() - min_pt.y();

// 计算 OccupancyGrid 的尺寸和分辨率
double resolution = 0.1; // 你的 Octree 的分辨率
double map_width = (max_pt.x() - min_pt.x()) / resolution;
double map_height = (max_pt.y() - min_pt.y()) / resolution;
double map_depth = (max_pt.z() - min_pt.z()) / resolution;
// 创建 OccupancyGrid 消息
nav_msgs::OccupancyGrid occupancy_grid;
occupancy_grid.header.stamp = ros::Time::now();
occupancy_grid.header.frame_id = "map";
occupancy_grid.info.resolution = resolution;
occupancy_grid.info.width = static_cast<uint32_t>(map_width);
occupancy_grid.info.height = static_cast<uint32_t>(map_height);
// occupancy_grid.info.depth = static_cast<uint32_t>(map_depth);
occupancy_grid.info.origin.position.x = min_pt.x();
occupancy_grid.info.origin.position.y = min_pt.y();
occupancy_grid.info.origin.position.z = min_pt.z();
occupancy_grid.data.resize(occupancy_grid.info.width * occupancy_grid.info.height, -1); // 初始化为未知
pcl::octree::OctreePointCloud<POINT_TYPE>::AlignedPointTVector voxel_centers;
// 遍历 Octree 中的每个点，将其映射到 OccupancyGrid
POINTCLOUD::Ptr cloud(new POINTCLOUD);
// octree.leaf_getBoundingBox(min_pt, max_pt);
octree.getOccupiedVoxelCenters(voxel_centers);
for (const auto& point : voxel_centers) {
    cloud->push_back(point);
}

for (const POINT_TYPE& point : cloud->points)
{
    int x = static_cast<int>((point.x - min_pt.x()) / resolution);
    int y = static_cast<int>((point.y - min_pt.y()) / resolution);
    int z = static_cast<int>((point.z - min_pt.z()) / resolution);

    if (x >= 0 && x < map_width && y >= 0 && y < map_height && z >= 0 && z < map_depth)
    {
        int index = z * (occupancy_grid.info.width * occupancy_grid.info.height) + y * occupancy_grid.info.width + x;
        occupancy_grid.data[index] = 100; // 设置占据的栅格
    }
}

// 发布 OccupancyGrid 消息
occupancy_grid_pub.publish(occupancy_grid);

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
    if(cloud_voxel_sum_queue.size() > 50) {
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
    occupancy_grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("occupancy_grid", 1);
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
        pcl::toROSMsg(cloud_voxel_sum, cloud_map_voxel_sum_msg);
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
