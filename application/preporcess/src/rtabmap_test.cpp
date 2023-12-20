#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <std_srvs/Empty.h>
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/conversions.h>   
#include <pcl_ros/transforms.h>
#include <queue>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
using POINTCLOUD_TYPE = pcl::PointXYZRGBNormal;
using POINTCLOUD = pcl::PointCloud<POINTCLOUD_TYPE>;
sensor_msgs::PointCloud2 cloud_map;
sensor_msgs::PointCloud2 cloud_voxel;
std::queue<POINTCLOUD> cloud_voxel_sum_queue;
POINTCLOUD cloud_voxel_sum;
// pcl::PointCloud<POINTCLOUD_TYPE> cloud_voxel_sum;
sensor_msgs::PointCloud2 cloud_map_voxel_sum;

void cloud_map_cb(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    cloud_map = *msg;
    ROS_INFO("Received point cloud with width: %d, height: %d", msg->width, msg->height);
}

void voxel_cb(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    cloud_voxel = *msg;

    POINTCLOUD cloud;
    pcl::fromROSMsg(cloud_voxel, cloud);
    cloud_voxel_sum_queue.push(cloud);
    if(cloud_voxel_sum_queue.size() > 15) {
        cloud_voxel_sum_queue.pop();
    }
    ROS_INFO("Received point cloud with width: %d, height: %d", msg->width, msg->height);
}

bool gen_sum_pc(){
    if(cloud_voxel_sum_queue.size() == 0) {
        ROS_INFO("cloud_voxel_sum is empty");
        return false;
    }
    ROS_INFO("save_cb");
    pcl::fromROSMsg(cloud_map, cloud_voxel_sum);
    // cloud += cloud_map;
    for(uint32_t i = 0; i < cloud_voxel_sum_queue.size(); i++) {
        cloud_voxel_sum += cloud_voxel_sum_queue.front();
        // cloud_voxel_sum.pop();
    }

#if 0
    pcl::PointCloud<POINTCLOUD_TYPE>::Ptr cloud_ptr(new pcl::PointCloud<POINTCLOUD_TYPE>(cloud_voxel_sum));
    pcl::PointCloud<POINTCLOUD_TYPE>::Ptr cloud_filtered(new pcl::PointCloud<POINTCLOUD_TYPE>);
    pcl::StatisticalOutlierRemoval<POINTCLOUD_TYPE> sor;
    sor.setInputCloud(cloud_ptr);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);
    cloud_voxel_sum = *cloud_filtered;
#endif

#if 0
    pcl::PointCloud<POINTCLOUD_TYPE>::Ptr cloud_ptr(new pcl::PointCloud<POINTCLOUD_TYPE>(cloud_voxel_sum));
    pcl::PointCloud<POINTCLOUD_TYPE>::Ptr cloud_filtered(new pcl::PointCloud<POINTCLOUD_TYPE>);
    pcl::VoxelGrid<POINTCLOUD_TYPE> sor;
    sor.setInputCloud(cloud_ptr);
    sor.setLeafSize(0.1, 0.1, 0.1);
    sor.filter(*cloud_filtered);
    cloud_voxel_sum = *cloud_filtered;
#endif

    return true;
    // pcl::toROSMsg(cloud_voxel_sum, cloud_map_voxel_sum);
}
bool save_pc_cb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
#if 0
    if(cloud_map.width*cloud_map.height == 0) {
        ROS_INFO("cloud_map is empty");
        return false;
    }
    ROS_INFO("save_cb");
    pcl::PointCloud<POINTCLOUD_TYPE> cloud;
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
    pcl::PointCloud<POINTCLOUD_TYPE> cloud;
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
        pcl::io::savePCDFileASCII("test_pcd.pcd", cloud_voxel_sum);
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

    ros::Subscriber sub_cloudmap = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_map", 1, cloud_map_cb);
    ros::Subscriber sub_voxel = nh.subscribe<sensor_msgs::PointCloud2>("/voxel", 1, voxel_cb);
    ros::Publisher  pub_cloudmap = nh.advertise<sensor_msgs::PointCloud2>("/cloud_map_sum", 1);
    // ros::Subscriber sub_voxel = nh.subscribe<sensor_msgs::PointCloud2>("/voxel", 1, voxel_cb);
    ros::ServiceServer service_savemap = nh.advertiseService("save_pc", save_pc_cb);
    ros::Rate rate(5);
    while (ros::ok())
    {
        gen_sum_pc();
        pcl::toROSMsg(cloud_voxel_sum, cloud_map_voxel_sum);
        pub_cloudmap.publish(cloud_map_voxel_sum);
        ros::spinOnce();
        ros::Duration(rate).sleep();
    }
    
    ros::spin();

    return 0;
}
